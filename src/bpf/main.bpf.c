/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2024 Andrea Righi <andrea.righi@linux.dev>
 *
 * scx_aura: laptop-oriented fork of scx_bpfland with XNU-style QoS.
 *
 * Tasks are classified into four QoS tiers that mirror the XNU Clutch
 * timeshare root buckets FG, DF, UT and BG. Each tier owns a shared DSQ
 * ordered by bpfland's virtual deadline. ops.dispatch() picks the tier to
 * consume from with the Clutch root-bucket algorithm
 * (sched_clutch_root_highest_root_bucket() in osfmk/kern/sched_clutch.c):
 *
 *  - EDF across tiers. A tier's deadline is the time it became runnable,
 *    or was last selected, plus its worst-case execution latency (WCEL).
 *  - A higher tier may run ahead of the EDF choice while it has warp
 *    budget left. The budget is refilled only when the tier is selected in
 *    natural priority order.
 *  - A lower tier that wins EDF while a higher tier is runnable gets a
 *    starvation-avoidance window of one tier quantum, after which its
 *    deadline moves out by its WCEL again.
 *
 * XNU's WCEL, warp and quantum tables keep their ratios but are scaled by
 * slice_max / 10 ms, so they stay proportional to this scheduler's slice.
 *
 * Tiers come from nice, SCHED_BATCH and SCHED_IDLE, then from behavior:
 * CPU burst length, wakeup rate and the thread group's XNU interactivity
 * score. A waking interactive task that finds no idle CPU preempts the CPU
 * running the lowest tier, big cores first.
 *
 * On hybrid CPUs, placement follows XNU's AMP policy: interactive, default
 * and utility work prefers big cores; background work, and utility work in
 * powersave mode, prefers little cores. Powersave keeps background work off
 * big cores entirely, and a big core takes background work only when no
 * other tier is waiting. At quantum expiry a misplaced task moves to an
 * idle core of the right class, or takes a big core from little-core work.
 * With schedutil, CPU frequency targets come from frequency-invariant
 * utilization, with a floor for interactive work and caps for utility and
 * background work, similar to what CLPC does with QoS on Apple platforms.
 */
#include <scx/common.bpf.h>
#include "intf.h"

#define STARVATION_MS		5000ULL
#define MAX_CPUS		1024
#define MAX_WAKEUP_FREQ		64ULL

#define TIER_MASK		(NR_TIERS - 1)
#define ALL_TIERS		((1U << NR_TIERS) - 1)
#define TIER_DSQ_BASE		0x100ULL

/* XNU root bucket tables (FG, DF, UT, BG), macOS thread quanta. */
#define XNU_REF_QUANTUM_US	10000ULL
#define WARP_UNUSED		(~0ULL)

/* Clutch bucket group interactivity score (sched_clutch.c). */
#define IACT_PRI		8
#define IACT_THRESH		12
#define IACT_BATCH_THRESH	4
#define IACT_WINDOW_NS		(500ULL * NSEC_PER_MSEC)
#define IACT_ADJUST_RATIO	10
#define IACT_MIN_HIST_NS	(IACT_WINDOW_NS / IACT_ADJUST_RATIO)

/* Wakeup frequency is sampled per 100 ms: 16 = 160 wakeups/s. */
#define IACT_WAKEUP_FREQ	16
#define BATCH_WAKEUP_FREQ	2

#define NICE_INTERACTIVE	(-5)
#define NICE_BACKGROUND		10

#define UTIL_WINDOW_NS		(4ULL * NSEC_PER_MSEC)
#define FP_ONE			1024U

#ifndef SCHED_BATCH
#define SCHED_BATCH		3
#endif
#ifndef SCHED_IDLE
#define SCHED_IDLE		5
#endif
#ifndef MAX_RT_PRIO
#define MAX_RT_PRIO		100
#endif

char _license[] SEC("license") = "GPL";

const volatile bool debug;
#define dbg_msg(_fmt, ...) do {				\
	if (debug)					\
		bpf_printk(_fmt, ##__VA_ARGS__);	\
} while (0)

/* Tunables set by user space before load. */
const volatile u64 slice_max = 1ULL * NSEC_PER_MSEC;
const volatile u64 slice_min;
const volatile u64 slice_lag = 40ULL * NSEC_PER_MSEC;
const volatile bool no_wake_sync;
const volatile bool sticky_tasks = true;
const volatile bool local_kthreads;
const volatile bool local_pcpu = true;
const volatile bool smt_enabled = true;
const volatile bool numa_enabled = true;
const volatile bool primary_all = true;
const volatile bool preferred_idle_scan;
const volatile bool hybrid;
const volatile bool group_iact_enabled = true;
const volatile bool warp_enabled = true;
const volatile u64 throttle_ns;

/* CPUs sorted by capacity, highest first, and the little-core flags. */
const volatile u64 preferred_cpus[MAX_CPUS];
const volatile u32 nr_preferred;
const volatile u8 cpu_little[MAX_CPUS];

/* TIMELY slice controller (Mittal et al., SIGCOMM 2015), opt-in. */
const volatile bool timely_enabled;
const volatile u64 timely_tlow_ns = 5ULL * NSEC_PER_MSEC;
const volatile u64 timely_thigh_ns = 50ULL * NSEC_PER_MSEC;
const volatile u32 timely_gain_min_fp = 128;
const volatile u32 timely_gain_step_fp = 32;
const volatile u32 timely_hai_thresh = 5;
const volatile u32 timely_hai_multiplier = 2;
const volatile u32 timely_beta_fp = 819;
const volatile u64 timely_gradient_margin_ns = 125ULL * NSEC_PER_USEC;
const volatile u64 timely_control_interval_ns = 500ULL * NSEC_PER_USEC;

/*
 * Power policy, rewritten by user space whenever the power profile
 * changes. Frequency values are fractions of a CPU's max frequency in
 * [0, 1024]; 0 disables the floor or cap.
 */
volatile s64 cpufreq_perf_lvl;
volatile u32 perf_iact_floor;
volatile u32 perf_util_cap;
volatile u32 perf_bg_cap;
volatile bool bg_strict;
volatile bool util_prefer_little;

/* Statistics. */
volatile u64 nr_running, nr_online_cpus;
volatile u64 nr_direct_dispatches, nr_local_dispatches, nr_keep_running,
	nr_preempt_kicks, nr_mig_up, nr_mig_down, nr_mig_smt,
	nr_promotions, nr_demotions, nr_reenq,
	nr_sel_natural, nr_sel_warp, nr_sel_starve_open, nr_sel_starve,
	nr_timely_inc, nr_timely_dec, nr_timely_hai;
volatile u64 nr_tier_enqueues[NR_TIERS], nr_tier_dispatches[NR_TIERS];
volatile s64 nr_tier_running[NR_TIERS];

UEI_DEFINE(uei);

private(AURA) struct bpf_cpumask __kptr *primary_cpumask;
private(AURA) struct bpf_cpumask __kptr *big_cpumask;
private(AURA) struct bpf_cpumask __kptr *little_cpumask;

static u64 nr_cpu_ids;

/*
 * Per-tier virtual clocks. Tiers are consumed from separate DSQs, so each
 * keeps its own fairness domain; a task changing tier is rebased.
 */
u64 vtime_now[NR_TIERS];

/* Per-tier Clutch parameters, computed in ops.init(). */
u64 tier_wcel_ns[NR_TIERS];
u64 tier_warp_ns[NR_TIERS];
u64 tier_quantum_ns[NR_TIERS];

/* Root bucket state, protected by root_lock. */
struct root_bucket {
	u64 deadline;
	u64 warp_remaining;
	u64 warped_deadline;
	u64 starvation_ts;
	u32 runnable;
	u32 warp_avail;
	u32 starving;
	u32 pad;
};

private(ROOT) struct bpf_spin_lock root_lock;
private(ROOT) struct root_bucket root_bkt[NR_TIERS];

enum sel_reason {
	SEL_NATURAL,
	SEL_WARP,
	SEL_STARVE_OPEN,
	SEL_STARVE,
};

static volatile bool cpus_throttled;

static inline bool is_throttled(void)
{
	return READ_ONCE(cpus_throttled);
}

static inline void set_throttled(bool state)
{
	WRITE_ONCE(cpus_throttled, state);
}

struct throttle_timer {
	struct bpf_timer timer;
};

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 1);
	__type(key, u32);
	__type(value, struct throttle_timer);
} throttle_timer SEC(".maps");

/*
 * Per-CPU context. cur_tier and mig_cpu are stored off by one so that the
 * zero-initialized state means "no SCX task running" and "no claim".
 * resume_pid names a task that gave up this CPU without losing its turn: a
 * task preempted by a waking interactive task, or one that yielded to a
 * lower tier's EDF or starvation turn. It goes back to this CPU's local DSQ
 * instead of the shared tier queue, keeping its cache, with the slice left
 * until slice_end. cur_pid is the running SCX task.
 */
struct cpu_ctx {
	u64 run_start;
	u64 busy_ns;
	u64 win_start;
	u64 slice_end;
	u32 util;
	u32 perf_target;
	s32 mig_cpu;
	s32 mig_pid;
	u32 cur_tier;
	s32 resume_pid;
	s32 cur_pid;
	u32 mig_preempt;	/* the claimed CPU is busy: preempt it */
	struct bpf_cpumask __kptr *smt;
};

struct {
	__uint(type, BPF_MAP_TYPE_PERCPU_ARRAY);
	__type(key, u32);
	__type(value, struct cpu_ctx);
	__uint(max_entries, 1);
} cpu_ctx_stor SEC(".maps");

static struct cpu_ctx *try_lookup_cpu_ctx(s32 cpu)
{
	const u32 idx = 0;

	return bpf_map_lookup_percpu_elem(&cpu_ctx_stor, &idx, cpu);
}

struct task_ctx {
	u64 awake_vtime;
	u64 last_run_at;	/* last runtime accounting point */
	u64 wakeup_freq;
	u64 last_woke_at;
	u64 burst;		/* CPU time since the last wakeup */
	u64 avg_burst;		/* average CPU time between wakeups */
	/* TIMELY */
	u64 enq_at;
	u64 prev_delay;
	s64 delay_grad;
	u64 gain_updated_at;
	u32 gain;
	u32 hai_count;
	/* Tier whose virtual clock p->scx.dsq_vtime refers to. */
	u32 tier;
	u32 pad;
};

struct {
	__uint(type, BPF_MAP_TYPE_TASK_STORAGE);
	__uint(map_flags, BPF_F_NO_PREALLOC);
	__type(key, int);
	__type(value, struct task_ctx);
} task_ctx_stor SEC(".maps");

static struct task_ctx *try_lookup_task_ctx(const struct task_struct *p)
{
	return bpf_task_storage_get(&task_ctx_stor, (struct task_struct *)p, 0, 0);
}

/*
 * Clutch bucket group equivalent, one per thread group. leader_start
 * detects tgid reuse; stale entries are replaced on the next wakeup and
 * dead ones age out of the LRU map.
 */
struct group_ctx {
	u64 leader_start;
	u64 cpu_used;
	u64 cpu_blocked;
	u64 blocked_since;
	s32 nr_runnable;
	u32 score;
	u32 hist;
	u32 pad;
};

struct {
	__uint(type, BPF_MAP_TYPE_LRU_HASH);
	__uint(max_entries, 8192);
	__type(key, u32);
	__type(value, struct group_ctx);
} group_map SEC(".maps");

/*
 * Bound a tier used as an array index. The barrier stops clang from
 * dropping a mask it considers redundant, which leaves the verifier with an
 * unbounded index once the value has been spilled to the stack.
 */
static __always_inline u32 tier_idx(u32 tier)
{
	barrier_var(tier);
	return tier & TIER_MASK;
}

static inline u64 tier_dsq(u32 tier)
{
	return TIER_DSQ_BASE + (tier & TIER_MASK);
}

/*
 * Same for indexes into the per-CPU rodata tables: clang may range-check
 * one copy of the value and index with another.
 */
static __always_inline u32 cpu_idx(u32 idx)
{
	barrier_var(idx);
	return idx & (MAX_CPUS - 1);
}

static inline bool is_kthread(const struct task_struct *p)
{
	return p->flags & PF_KTHREAD;
}

static inline bool is_task_queued(const struct task_struct *p)
{
	return p->scx.flags & SCX_TASK_QUEUED;
}

static inline bool is_pcpu_task(const struct task_struct *p)
{
	return p->nr_cpus_allowed == 1 || is_migration_disabled(p);
}

static inline bool cpu_is_little(s32 cpu)
{
	if (!hybrid || (u32)cpu >= MAX_CPUS)
		return false;
	return cpu_little[cpu_idx(cpu)];
}

static inline bool tier_prefers_little(u32 tier)
{
	return tier == TIER_BACKGROUND ||
	       (tier == TIER_UTILITY && util_prefer_little);
}

/* Tiers a CPU may consume: strict mode keeps background work off big cores. */
static inline u32 tier_eligible(s32 cpu)
{
	if (hybrid && bg_strict && !cpu_is_little(cpu))
		return ALL_TIERS & ~(1U << TIER_BACKGROUND);
	return ALL_TIERS;
}

static inline u64 calc_avg(u64 old_val, u64 new_val)
{
	return (old_val - (old_val >> 2)) + (new_val >> 2);
}

static inline u64 update_freq(u64 freq, u64 interval)
{
	return calc_avg(freq, (100ULL * NSEC_PER_MSEC) / interval);
}

/*
 * CPU time per burst, counting the burst in progress: a task that never
 * sleeps has no completed bursts, but must not look short.
 */
static inline u64 task_burst(const struct task_ctx *tctx)
{
	return MAX(tctx->avg_burst, tctx->burst);
}

static inline const struct cpumask *get_idle_cpumask(s32 cpu)
{
	if (!numa_enabled)
		return scx_bpf_get_idle_cpumask();
	return __COMPAT_scx_bpf_get_idle_cpumask_node(__COMPAT_scx_bpf_cpu_node(cpu));
}

static inline const struct cpumask *get_idle_smtmask(s32 cpu)
{
	if (!numa_enabled)
		return scx_bpf_get_idle_smtmask();
	return __COMPAT_scx_bpf_get_idle_smtmask_node(__COMPAT_scx_bpf_cpu_node(cpu));
}

/* Return the SMT sibling of @cpu, or -1 when it has none. */
static s32 smt_sibling(s32 cpu)
{
	struct cpu_ctx *cctx = try_lookup_cpu_ctx(cpu);
	const struct cpumask *smt;
	u32 sib;

	if (!cctx)
		return -1;
	smt = cast_mask(cctx->smt);
	if (!smt)
		return -1;
	sib = bpf_cpumask_first(smt);
	if (sib >= nr_cpu_ids || sib == (u32)cpu)
		return -1;
	return sib;
}

static u64 nr_tasks_waiting(void)
{
	return scx_bpf_dsq_nr_queued(tier_dsq(TIER_INTERACTIVE)) +
	       scx_bpf_dsq_nr_queued(tier_dsq(TIER_DEFAULT)) +
	       scx_bpf_dsq_nr_queued(tier_dsq(TIER_UTILITY)) +
	       scx_bpf_dsq_nr_queued(tier_dsq(TIER_BACKGROUND));
}

static u64 task_slice(const struct task_struct *p, const struct task_ctx *tctx)
{
	u64 slice = scale_by_task_weight(p, slice_max) / MAX(nr_tasks_waiting(), 1);

	if (timely_enabled && tctx)
		slice = slice * tctx->gain / FP_ONE;

	return MAX(CLAMP(slice, slice_min, slice_max), NSEC_PER_USEC);
}

/*
 * bpfland's virtual deadline, evaluated against the clock of @tier:
 * deadline = vruntime + vruntime accumulated since the last wakeup.
 */
static u64 task_dl(struct task_struct *p, struct task_ctx *tctx, u32 tier)
{
	const u64 STARVATION_THRESH = STARVATION_MS * NSEC_PER_MSEC / 10;
	const u64 q_thresh = MAX(STARVATION_THRESH / slice_max, 1);
	u64 nr_queued = scx_bpf_dsq_nr_queued(tier_dsq(tier));
	u64 lag_scale = MAX(tctx->wakeup_freq, 1);
	u64 awake_max = scale_by_task_weight_inverse(p, slice_lag);
	u64 vtime_min;

	if (nr_queued * slice_max >= STARVATION_THRESH)
		lag_scale = 1;
	else
		lag_scale = MAX(lag_scale * q_thresh / (q_thresh + nr_queued), 1);

	vtime_min = vtime_now[tier_idx(tier)] -
		    scale_by_task_weight(p, slice_lag * lag_scale);
	if (time_before(p->scx.dsq_vtime, vtime_min))
		scx_bpf_task_set_dsq_vtime(p, vtime_min);

	if (time_after(tctx->awake_vtime, awake_max))
		tctx->awake_vtime = awake_max;

	return p->scx.dsq_vtime + tctx->awake_vtime;
}

static struct group_ctx *group_lookup(const struct task_struct *p, bool create)
{
	struct group_ctx *g, init = {};
	u32 tgid = p->tgid;
	u64 start = BPF_CORE_READ(p, group_leader, start_time);

	g = bpf_map_lookup_elem(&group_map, &tgid);
	if (g && g->leader_start == start)
		return g;
	if (!create)
		return NULL;

	/*
	 * Like XNU, a new thread group starts fully interactive (500 ms
	 * blocked, no CPU used), so a newly launched program competes fairly
	 * with running ones until its own behavior takes over.
	 */
	init.leader_start = start;
	init.cpu_blocked = IACT_WINDOW_NS;
	init.score = 2 * IACT_PRI;
	init.hist = 1;
	/* NOEXIST: a sibling thread may have created it concurrently. */
	bpf_map_update_elem(&group_map, &tgid, &init, g ? BPF_ANY : BPF_NOEXIST);
	g = bpf_map_lookup_elem(&group_map, &tgid);
	return g && g->leader_start == start ? g : NULL;
}

/*
 * XNU interactivity score: 8..16 when the group spends more time blocked
 * than running, 0..8 otherwise. Once CPU used plus blocked time reaches
 * 500 ms both are divided by 10, keeping only recent behavior.
 */
static void group_refresh(struct group_ctx *g)
{
	u64 used = g->cpu_used, blocked = g->cpu_blocked;

	if (used + blocked >= IACT_WINDOW_NS) {
		used /= IACT_ADJUST_RATIO;
		blocked /= IACT_ADJUST_RATIO;
		g->cpu_used = used;
		g->cpu_blocked = blocked;
	}
	g->hist = used + blocked >= IACT_MIN_HIST_NS;

	if (!used && !blocked)
		g->score = IACT_PRI;
	else if (blocked > used)
		g->score = IACT_PRI + IACT_PRI * (blocked - used) / blocked;
	else
		g->score = IACT_PRI * blocked / used;
}

static void group_runnable(const struct task_struct *p, u64 now)
{
	struct group_ctx *g;
	u64 since;
	s32 old;

	if (!group_iact_enabled)
		return;
	g = group_lookup(p, true);
	if (!g)
		return;

	old = __sync_fetch_and_add(&g->nr_runnable, 1);
	if (old > 0)
		return;
	if (old < 0)
		g->nr_runnable = 1;

	since = g->blocked_since;
	g->blocked_since = 0;
	if (since && now > since) {
		__sync_fetch_and_add(&g->cpu_blocked, MIN(now - since, IACT_WINDOW_NS));
		group_refresh(g);
	}
}

static void group_quiescent(const struct task_struct *p, u64 now)
{
	struct group_ctx *g;
	s32 old;

	if (!group_iact_enabled)
		return;
	g = group_lookup(p, false);
	if (!g)
		return;

	old = __sync_fetch_and_sub(&g->nr_runnable, 1);
	if (old == 1)
		g->blocked_since = now;
	else if (old <= 0)
		__sync_fetch_and_add(&g->nr_runnable, 1);
}

static void group_charge(const struct task_struct *p, u64 runtime)
{
	struct group_ctx *g;

	if (!group_iact_enabled || !runtime)
		return;
	g = group_lookup(p, false);
	if (!g)
		return;

	__sync_fetch_and_add(&g->cpu_used, MIN(runtime, IACT_WINDOW_NS));
	group_refresh(g);
}

static bool can_use_little(const struct task_struct *p)
{
	const struct cpumask *little = cast_mask(little_cpumask);

	return little && bpf_cpumask_intersects(p->cpus_ptr, little);
}

/*
 * Linux has no QoS classes, so the tier is derived from explicit hints
 * (nice, SCHED_BATCH, SCHED_IDLE) first, then from behavior: short bursts
 * with frequent wakeups, or a mostly blocked thread group, qualify as
 * interactive; long bursts with rare wakeups in a CPU-bound group are
 * demoted to utility. A burst is the CPU time between two wakeups, so
 * preemption does not make a CPU-bound task look bursty. The thresholds
 * for staying in a behavioral tier are looser than those for entering it,
 * so tasks close to a threshold do not change tier on every sample.
 */
static u32 task_classify(const struct task_struct *p, const struct task_ctx *tctx)
{
	s32 nice = (s32)p->static_prio - 120;
	u64 runtime = task_burst(tctx);
	u64 freq = tctx->wakeup_freq;
	u32 cur = tctx->tier & TIER_MASK;
	struct group_ctx *g = NULL;
	bool hist = false;
	u32 score = IACT_PRI;

	if (p->policy == SCHED_IDLE || nice >= NICE_BACKGROUND) {
		if (hybrid && !can_use_little(p))
			return TIER_UTILITY;
		return TIER_BACKGROUND;
	}
	if (nice < NICE_INTERACTIVE)
		return TIER_INTERACTIVE;
	if (p->policy == SCHED_BATCH)
		return TIER_UTILITY;

	if (group_iact_enabled)
		g = group_lookup(p, false);
	if (g && g->hist) {
		hist = true;
		score = g->score;
	}

	if (cur == TIER_INTERACTIVE) {
		if (runtime <= slice_max &&
		    (freq >= IACT_WAKEUP_FREQ / 2 || (hist && score >= IACT_THRESH - 2)))
			return TIER_INTERACTIVE;
	} else if (runtime <= slice_max / 2 &&
		   (freq >= IACT_WAKEUP_FREQ || (hist && score >= IACT_THRESH))) {
		return TIER_INTERACTIVE;
	}

	if (cur == TIER_UTILITY) {
		if (hist && score <= IACT_BATCH_THRESH + 2 && runtime >= slice_max / 2 &&
		    freq < 2 * BATCH_WAKEUP_FREQ)
			return TIER_UTILITY;
	} else if (hist && score <= IACT_BATCH_THRESH && runtime >= slice_max &&
		   freq < BATCH_WAKEUP_FREQ) {
		return TIER_UTILITY;
	}

	return TIER_DEFAULT;
}

/*
 * Refresh the tier of @p. Only called while @p is outside any DSQ, so its
 * vtime can be rebased onto the new tier's clock, keeping its lag.
 */
static u32 task_update_tier(struct task_struct *p, struct task_ctx *tctx)
{
	u32 old = tctx->tier & TIER_MASK;
	u32 tier = task_classify(p, tctx) & TIER_MASK;
	s64 lag, lim;

	if (tier == old)
		return tier;

	lag = (s64)(p->scx.dsq_vtime - vtime_now[tier_idx(old)]);
	lim = (s64)scale_by_task_weight_inverse(p, slice_lag);
	if (lag > lim)
		lag = lim;
	else if (lag < -lim)
		lag = -lim;
	scx_bpf_task_set_dsq_vtime(p, vtime_now[tier_idx(tier)] + lag);
	tctx->tier = tier;

	if (tier < old)
		__sync_fetch_and_add(&nr_promotions, 1);
	else
		__sync_fetch_and_add(&nr_demotions, 1);

	return tier;
}

static __always_inline void rb_runnable(u32 tier, u64 now)
{
	struct root_bucket *rb = &root_bkt[tier_idx(tier)];

	rb->runnable = 1;
	if (!rb->starving)
		rb->deadline = now + tier_wcel_ns[tier_idx(tier)];
	rb->warp_avail = rb->warp_remaining > 0;
}

/*
 * XNU keeps the unused part of an open warp window when a bucket empties.
 * The window is also reset to unused here, so the saved budget can be
 * spent the next time the tier becomes runnable.
 */
static __always_inline void rb_empty(u32 tier, u64 now)
{
	struct root_bucket *rb = &root_bkt[tier_idx(tier)];

	rb->runnable = 0;
	rb->warp_avail = 0;
	if (rb->warped_deadline != WARP_UNUSED) {
		rb->warp_remaining = rb->warped_deadline > now ?
				     rb->warped_deadline - now : 0;
		rb->warped_deadline = WARP_UNUSED;
	}
}

static __always_inline void rb_reconcile(u32 qmask, u64 now)
{
	u32 t;

#pragma unroll
	for (t = 0; t < NR_TIERS; t++) {
		bool queued = qmask & (1U << t);

		if (queued && !root_bkt[t].runnable)
			rb_runnable(t, now);
		else if (!queued && root_bkt[t].runnable)
			rb_empty(t, now);
	}
}

/*
 * Clutch root bucket selection over the tiers in @cand. Expired starvation
 * and warp windows are closed up front; XNU closes them lazily when the
 * bucket is next considered, which yields the same order because a
 * starving bucket keeps the earliest deadline until its window closes.
 */
static __always_inline s32 rb_select(u32 cand, u64 now, u32 *how)
{
	struct root_bucket *rb;
	s32 edf = -1, warp = -1, highest = -1;
	u64 dl = ~0ULL;
	u32 t;

#pragma unroll
	for (t = 0; t < NR_TIERS; t++) {
		rb = &root_bkt[t];
		if (rb->starving && now >= rb->starvation_ts + tier_quantum_ns[t]) {
			rb->starving = 0;
			rb->deadline = now + tier_wcel_ns[t];
		}
		if (rb->warped_deadline != WARP_UNUSED && rb->warped_deadline <= now) {
			rb->warp_remaining = 0;
			rb->warp_avail = 0;
		}
		if (!(cand & (1U << t)))
			continue;
		if (highest < 0)
			highest = t;
		if (rb->deadline < dl) {
			dl = rb->deadline;
			edf = t;
		}
	}
	if (edf < 0)
		return -1;

	/* An open starvation window takes precedence over warp, as in XNU. */
	rb = &root_bkt[tier_idx(edf)];
	if (rb->starving) {
		*how = SEL_STARVE;
		return edf;
	}
	if (highest == edf) {
		rb->deadline = now + tier_wcel_ns[tier_idx(edf)];
		rb->warp_remaining = tier_warp_ns[tier_idx(edf)];
		rb->warped_deadline = WARP_UNUSED;
		rb->warp_avail = rb->runnable && rb->warp_remaining;
		*how = SEL_NATURAL;
		return edf;
	}

#pragma unroll
	for (t = 0; t < NR_TIERS - 1; t++) {
		if ((s32)t >= edf)
			break;
		if ((cand & (1U << t)) && root_bkt[t].warp_avail) {
			warp = t;
			break;
		}
	}

	if (warp < 0) {
		rb->starving = 1;
		rb->starvation_ts = now;
		*how = SEL_STARVE_OPEN;
		return edf;
	}

	rb = &root_bkt[tier_idx(warp)];
	if (rb->warped_deadline == WARP_UNUSED)
		rb->warped_deadline = now + rb->warp_remaining;
	rb->deadline = now + tier_wcel_ns[tier_idx(warp)];
	*how = SEL_WARP;
	return warp;
}

static __always_inline void tier_mark_runnable(u32 tier, u64 now)
{
	tier = tier_idx(tier);
	if (READ_ONCE(root_bkt[tier].runnable))
		return;
	bpf_spin_lock(&root_lock);
	if (!root_bkt[tier].runnable)
		rb_runnable(tier, now);
	bpf_spin_unlock(&root_lock);
}

#define SCAN_FULL	(1U << 0)	/* whole core idle */
#define SCAN_PARTIAL	(1U << 1)	/* SMT sibling busy, core already awake */
#define SCAN_ASC	(1U << 2)	/* lowest capacity first */

static __always_inline bool cpu_ok(const struct task_struct *p, s32 cpu,
				   const struct cpumask *idle, const struct cpumask *filter,
				   const struct cpumask *smt, u32 flags)
{
	if (!bpf_cpumask_test_cpu(cpu, idle) || !bpf_cpumask_test_cpu(cpu, p->cpus_ptr))
		return false;
	if (filter && !bpf_cpumask_test_cpu(cpu, filter))
		return false;
	if (smt && (flags & SCAN_FULL) && !bpf_cpumask_test_cpu(cpu, smt))
		return false;
	if (smt && (flags & SCAN_PARTIAL) && bpf_cpumask_test_cpu(cpu, smt))
		return false;
	return true;
}

/*
 * Claim the first idle CPU in @filter, walking CPUs by capacity (highest
 * first unless SCAN_ASC). @prev_cpu is tried first; @ref_cpu selects the
 * NUMA node whose idle masks are used.
 */
static s32 scan_idle(const struct task_struct *p, s32 prev_cpu, s32 ref_cpu,
		     const struct cpumask *filter, u32 flags)
{
	const struct cpumask *idle, *smt = NULL;
	bool asc = flags & SCAN_ASC;
	s32 cpu = -EBUSY;
	u32 i;

	idle = get_idle_cpumask(ref_cpu);
	if (smt_enabled && (flags & (SCAN_FULL | SCAN_PARTIAL)))
		smt = get_idle_smtmask(ref_cpu);

	if (prev_cpu >= 0 && cpu_ok(p, prev_cpu, idle, filter, smt, flags) &&
	    scx_bpf_test_and_clear_cpu_idle(prev_cpu)) {
		cpu = prev_cpu;
		goto out;
	}

	bpf_for(i, 0, nr_preferred) {
		u32 idx = asc ? nr_preferred - 1 - i : i;
		s32 c;

		if (idx >= MAX_CPUS)
			break;
		c = preferred_cpus[cpu_idx(idx)];
		if (c == prev_cpu || !cpu_ok(p, c, idle, filter, smt, flags))
			continue;
		if (scx_bpf_test_and_clear_cpu_idle(c)) {
			cpu = c;
			break;
		}
	}
out:
	if (smt)
		scx_bpf_put_cpumask(smt);
	scx_bpf_put_cpumask(idle);
	return cpu;
}

/* Idle CPU selection through the kernel's topology-aware helper. */
static s32 pick_idle_cpu_kernel(struct task_struct *p, s32 prev_cpu, u64 wake_flags,
				bool eff, bool strict)
{
	const struct cpumask *first = NULL, *second = NULL;
	s32 cpu;

	if (no_wake_sync)
		wake_flags &= ~SCX_WAKE_SYNC;

	if (hybrid) {
		if (eff) {
			first = cast_mask(little_cpumask);
		} else {
			first = primary_all ? cast_mask(big_cpumask) : cast_mask(primary_cpumask);
			second = cast_mask(little_cpumask);
		}
	} else if (!eff && !primary_all) {
		first = cast_mask(primary_cpumask);
	}

	if (first) {
		cpu = scx_bpf_select_cpu_and(p, prev_cpu, wake_flags, first, 0);
		if (cpu >= 0)
			return cpu;
	}
	if (second) {
		cpu = scx_bpf_select_cpu_and(p, prev_cpu, wake_flags, second, 0);
		if (cpu >= 0)
			return cpu;
	}
	if (strict)
		return -EBUSY;
	return scx_bpf_select_cpu_and(p, prev_cpu, wake_flags, p->cpus_ptr, 0);
}

static bool any_idle_cpu(const struct task_struct *p, s32 ref_cpu)
{
	const struct cpumask *idle = get_idle_cpumask(ref_cpu);
	bool ret = bpf_cpumask_intersects(idle, p->cpus_ptr);

	scx_bpf_put_cpumask(idle);
	return ret;
}

/*
 * Pick and claim an idle CPU for @p according to its tier.
 *
 * Performance-first on hybrid CPUs: whole idle big core, then little core,
 * then anything left (big-core SMT siblings). Efficiency-first: little
 * core, then (unless strict) an idle SMT sibling of a busy core, whose core
 * is already awake, then anything, lowest capacity first.
 */
static s32 pick_idle_cpu(struct task_struct *p, s32 prev_cpu, u64 wake_flags, u32 tier)
{
	const struct cpumask *perf, *little = NULL;
	bool eff = tier_prefers_little(tier);
	bool strict = hybrid && tier == TIER_BACKGROUND && bg_strict;
	s32 cpu = -EBUSY;

	if (!preferred_idle_scan && __COMPAT_HAS_scx_bpf_select_cpu_and)
		return pick_idle_cpu_kernel(p, prev_cpu, wake_flags, eff, strict);

	if (!any_idle_cpu(p, prev_cpu))
		return -EBUSY;

	if (!primary_all)
		perf = cast_mask(primary_cpumask);
	else
		perf = hybrid ? cast_mask(big_cpumask) : NULL;
	if (hybrid)
		little = cast_mask(little_cpumask);

	if (hybrid && !eff) {
		cpu = scan_idle(p, prev_cpu, prev_cpu, perf, SCAN_FULL);
		if (cpu < 0)
			cpu = scan_idle(p, prev_cpu, prev_cpu, little, 0);
		if (cpu < 0)
			cpu = scan_idle(p, prev_cpu, prev_cpu, NULL, 0);
	} else if (hybrid) {
		cpu = scan_idle(p, prev_cpu, prev_cpu, little, SCAN_ASC);
		if (cpu < 0 && !strict)
			cpu = scan_idle(p, prev_cpu, prev_cpu, NULL, SCAN_PARTIAL | SCAN_ASC);
		if (cpu < 0 && !strict)
			cpu = scan_idle(p, prev_cpu, prev_cpu, NULL, SCAN_ASC);
	} else if (!eff) {
		if (perf) {
			cpu = scan_idle(p, prev_cpu, prev_cpu, perf, SCAN_FULL);
			if (cpu < 0)
				cpu = scan_idle(p, prev_cpu, prev_cpu, perf, 0);
		}
		if (cpu < 0)
			cpu = scan_idle(p, prev_cpu, prev_cpu, NULL, SCAN_FULL);
		if (cpu < 0)
			cpu = scan_idle(p, prev_cpu, prev_cpu, NULL, 0);
	} else {
		cpu = scan_idle(p, prev_cpu, prev_cpu, NULL, SCAN_PARTIAL | SCAN_ASC);
		if (cpu < 0)
			cpu = scan_idle(p, prev_cpu, prev_cpu, NULL, SCAN_ASC);
	}

	return cpu;
}

enum mig_reason {
	MIG_UP,
	MIG_DOWN,
	MIG_SMT,
};

static u32 cpu_cur_tier(s32 cpu);

/* A big core running work that prefers little cores, lowest tier first. */
static s32 swap_target(const struct task_struct *p)
{
	s32 target = -EBUSY;
	u32 i, cur, best = 0;

	bpf_for(i, 0, nr_preferred) {
		s32 c;

		if (i >= MAX_CPUS)
			break;
		c = preferred_cpus[cpu_idx(i)];
		/* The CPU order lists big cores first. */
		if (cpu_is_little(c))
			break;
		if (!bpf_cpumask_test_cpu(c, p->cpus_ptr))
			continue;
		cur = cpu_cur_tier(c);
		if (cur > best && tier_prefers_little(cur - 1)) {
			best = cur;
			target = c;
		}
	}
	return target;
}

/*
 * Quantum-expiry rebalancing (XNU AMP/Edge): claim an idle CPU of the core
 * class @p's tier prefers, or a whole idle core when @cpu's SMT sibling is
 * busy. ops.enqueue() then moves @p there. Work that prefers big cores and
 * finds none idle takes a big core running little-core work instead, whose
 * task goes back to its tier queue for the little cores to pick up; *swap
 * is set then.
 */
static s32 claim_better_cpu(struct task_struct *p, s32 cpu, u32 tier, u32 *reason,
			    bool *swap)
{
	const struct cpumask *idle, *smt, *same = NULL;
	bool little = cpu_is_little(cpu);
	bool eff = tier_prefers_little(tier);
	bool contended = false;
	s32 target = -EBUSY, sib;

	if (!any_idle_cpu(p, cpu)) {
		if (hybrid && little && !eff) {
			target = swap_target(p);
			*swap = target >= 0;
			*reason = MIG_UP;
		}
		return target;
	}

	if (hybrid && little && !eff) {
		target = scan_idle(p, -1, cpu, cast_mask(big_cpumask), SCAN_FULL);
		if (target < 0) {
			target = swap_target(p);
			*swap = target >= 0;
		}
		*reason = MIG_UP;
		if (target >= 0)
			return target;
	} else if (hybrid && !little && eff) {
		target = scan_idle(p, -1, cpu, cast_mask(little_cpumask), SCAN_ASC);
		*reason = MIG_DOWN;
	}
	if (target >= 0 || !smt_enabled)
		return target;

	sib = smt_sibling(cpu);
	if (sib < 0)
		return -EBUSY;
	idle = get_idle_cpumask(cpu);
	smt = get_idle_smtmask(cpu);
	contended = !bpf_cpumask_test_cpu(sib, idle) && !bpf_cpumask_empty(smt);
	scx_bpf_put_cpumask(smt);
	scx_bpf_put_cpumask(idle);
	if (!contended)
		return -EBUSY;

	if (hybrid)
		same = little ? cast_mask(little_cpumask) : cast_mask(big_cpumask);
	*reason = MIG_SMT;
	return scan_idle(p, -1, cpu, same, SCAN_FULL | (eff ? SCAN_ASC : 0));
}

/*
 * Decide whether @p, whose slice expired on @cpu, keeps running there.
 * When it should move, a claimed target is left in the CPU context for
 * ops.enqueue(). @elig is the set of tiers @cpu may run right now.
 */
static bool prev_stays(struct task_struct *p, struct task_ctx *tctx, s32 cpu,
		       struct cpu_ctx *cctx, u32 elig)
{
	u32 tier = tctx->tier & TIER_MASK, reason = MIG_SMT;
	bool swap = false;
	s32 target;

	if (is_pcpu_task(p))
		return true;

	target = claim_better_cpu(p, cpu, tier, &reason, &swap);
	if (target >= 0) {
		cctx->mig_cpu = target + 1;
		cctx->mig_pid = p->pid;
		cctx->mig_preempt = swap;
		if (reason == MIG_UP)
			__sync_fetch_and_add(&nr_mig_up, 1);
		else if (reason == MIG_DOWN)
			__sync_fetch_and_add(&nr_mig_down, 1);
		else
			__sync_fetch_and_add(&nr_mig_smt, 1);
		return false;
	}

	return elig & (1U << tier);
}

/* A claim that ops.enqueue() did not consume: give the CPU back. */
static void release_stale_claim(struct cpu_ctx *cctx)
{
	s32 cpu = cctx->mig_cpu - 1;

	if (cpu < 0)
		return;
	cctx->mig_cpu = 0;
	if (!cctx->mig_preempt)
		scx_bpf_kick_cpu(cpu, SCX_KICK_IDLE);
	cctx->mig_preempt = 0;
}

/*
 * Utilization tracking for cpufreq. Busy time is accumulated per CPU and
 * folded into a frequency-invariant average every UTIL_WINDOW_NS: the
 * average rises fast and decays in proportion to elapsed time.
 */
static void util_account(struct cpu_ctx *cctx, u64 now)
{
	if (cctx->run_start && now > cctx->run_start)
		cctx->busy_ns += now - cctx->run_start;
	cctx->run_start = now;
}

static void util_update(s32 cpu, struct cpu_ctx *cctx, u64 now)
{
	u64 elapsed, busy, sample, cur = SCX_CPUPERF_ONE, n;

	if (!cctx->win_start || now < cctx->win_start) {
		cctx->win_start = now;
		cctx->busy_ns = 0;
		return;
	}
	elapsed = now - cctx->win_start;
	if (elapsed < UTIL_WINDOW_NS)
		return;

	busy = MIN(cctx->busy_ns, elapsed);
	if (bpf_ksym_exists(scx_bpf_cpuperf_cur))
		cur = scx_bpf_cpuperf_cur(cpu);
	sample = busy * cur / elapsed;

	/* Saturated at the current level: demand is at least that, ramp up. */
	if (busy * 8 >= elapsed * 7)
		sample = MAX(sample, (cur + SCX_CPUPERF_ONE) / 2);

	if (sample >= cctx->util) {
		cctx->util += (sample - cctx->util) / 2;
	} else {
		n = MIN(elapsed / UTIL_WINDOW_NS, 8);
		cctx->util -= (cctx->util - sample) * n / 8;
	}
	cctx->busy_ns = 0;
	cctx->win_start = now;
}

/*
 * Set the cpuperf target of @cpu for a task of @tier. schedutil treats the
 * target as utilization in capacity units and adds 25% headroom, so
 * utilization is scaled by the CPU capacity, and frequency floors and caps
 * are taken at 4/5.
 */
static void perf_update(s32 cpu, struct cpu_ctx *cctx, u32 tier, u64 now)
{
	u64 target, cap = SCX_CPUPERF_ONE;
	u32 floor = perf_iact_floor, util_cap = perf_util_cap, bg_cap = perf_bg_cap;

	if (cpufreq_perf_lvl >= 0)
		return;

	util_update(cpu, cctx, now);
	target = cctx->util;

	if (tier == TIER_INTERACTIVE && floor)
		target = MAX(target, floor * 4 / 5);
	else if (tier == TIER_UTILITY && util_cap)
		target = MIN(target, util_cap * 4 / 5);
	else if (tier == TIER_BACKGROUND && bg_cap)
		target = MIN(target, bg_cap * 4 / 5);

	if (bpf_ksym_exists(scx_bpf_cpuperf_cap))
		cap = scx_bpf_cpuperf_cap(cpu);
	target = MIN(target * cap / SCX_CPUPERF_ONE, SCX_CPUPERF_ONE);

	if (target != cctx->perf_target) {
		cctx->perf_target = target;
		scx_bpf_cpuperf_set(cpu, target);
	}
}

/*
 * TIMELY: per-task gain on the time slice, driven by queueing delay.
 * Additive increase below tlow, multiplicative decrease above thigh, and
 * gradient-based control in between with hyperactive increase (HAI) after
 * timely_hai_thresh consecutive non-rising samples.
 */
static void timely_sample(struct task_ctx *tctx, u64 now)
{
	u64 delay, f, ng;
	s64 diff, grad;
	u32 gain, n;

	if (!tctx->enq_at || now <= tctx->enq_at) {
		tctx->enq_at = 0;
		return;
	}
	delay = now - tctx->enq_at;
	tctx->enq_at = 0;

	diff = (s64)delay - (s64)tctx->prev_delay;
	tctx->prev_delay = delay;
	tctx->delay_grad += (diff - tctx->delay_grad) >> 2;

	if (now - tctx->gain_updated_at < timely_control_interval_ns)
		return;
	tctx->gain_updated_at = now;

	gain = tctx->gain;
	grad = tctx->delay_grad;

	if (delay < timely_tlow_ns) {
		gain += timely_gain_step_fp;
		tctx->hai_count = 0;
		__sync_fetch_and_add(&nr_timely_inc, 1);
	} else if (delay > timely_thigh_ns) {
		f = (u64)timely_beta_fp * (delay - timely_thigh_ns) / delay;
		gain -= (u32)((u64)gain * f / FP_ONE);
		tctx->hai_count = 0;
		__sync_fetch_and_add(&nr_timely_dec, 1);
	} else if (grad <= 0) {
		n = ++tctx->hai_count >= timely_hai_thresh ? timely_hai_multiplier : 1;
		gain += n * timely_gain_step_fp;
		__sync_fetch_and_add(n > 1 ? &nr_timely_hai : &nr_timely_inc, 1);
	} else {
		ng = MIN((u64)grad * FP_ONE / MAX(timely_gradient_margin_ns, 1), FP_ONE);
		f = (u64)timely_beta_fp * ng / FP_ONE;
		gain -= (u32)((u64)gain * f / FP_ONE);
		tctx->hai_count = 0;
		__sync_fetch_and_add(&nr_timely_dec, 1);
	}

	tctx->gain = CLAMP(gain, MAX(timely_gain_min_fp, 1), FP_ONE);
}

static bool is_task_sticky(const struct task_ctx *tctx)
{
	return sticky_tasks && task_burst(tctx) < 10 * NSEC_PER_USEC;
}

static bool try_direct(struct task_struct *p, s32 prev_cpu, u32 tier, u64 slice, u64 enq_flags)
{
	s32 cpu = pick_idle_cpu(p, prev_cpu, 0, tier);

	if (cpu < 0)
		return false;
	scx_bpf_dsq_insert(p, SCX_DSQ_LOCAL_ON | cpu, slice, enq_flags);
	scx_bpf_kick_cpu(cpu, SCX_KICK_IDLE);
	__sync_fetch_and_add(&nr_direct_dispatches, 1);
	return true;
}

/* Track which tier @cctx's CPU is running (0 = none) in nr_tier_running. */
static void set_cur_tier(struct cpu_ctx *cctx, u32 cur)
{
	u32 old = cctx->cur_tier;

	if (old == cur)
		return;
	if (old)
		__sync_fetch_and_sub(&nr_tier_running[tier_idx(old - 1)], 1);
	if (cur)
		__sync_fetch_and_add(&nr_tier_running[tier_idx(cur - 1)], 1);
	WRITE_ONCE(cctx->cur_tier, cur);
}

static u32 cpu_cur_tier(s32 cpu)
{
	struct cpu_ctx *cctx = try_lookup_cpu_ctx(cpu);

	return cctx ? READ_ONCE(cctx->cur_tier) : 0;
}

/*
 * CPU for a waking interactive task that found no idle CPU: @prev_cpu if it
 * runs a lower tier, otherwise the CPU running the lowest tier (XNU's
 * processor choice: last processor if it runs lower-priority work,
 * otherwise the lowest-priority processor). On hybrid CPUs big cores are
 * tried first, as XNU keeps foreground work on the performance cluster. A
 * CPU that went idle since the idle scan is claimed and returned with @idle
 * set. Returns -1 when every allowed CPU runs interactive work.
 */
static s32 preempt_target(const struct task_struct *p, s32 prev_cpu, bool *idle)
{
	s32 victim = -1;
	u32 worst = TIER_INTERACTIVE, tier, i;

	tier = cpu_cur_tier(prev_cpu);
	if (tier > TIER_INTERACTIVE + 1 && !cpu_is_little(prev_cpu) &&
	    bpf_cpumask_test_cpu(prev_cpu, p->cpus_ptr))
		return prev_cpu;

	bpf_for(i, 0, nr_preferred) {
		s32 cpu;

		if (i >= MAX_CPUS)
			break;
		cpu = preferred_cpus[cpu_idx(i)];
		/* The CPU order lists big cores first. */
		if (victim >= 0 && cpu_is_little(cpu) && !cpu_is_little(victim))
			break;
		if (!bpf_cpumask_test_cpu(cpu, p->cpus_ptr))
			continue;
		tier = cpu_cur_tier(cpu);
		if (!tier) {
			/* Not running SCX work: take it if it is really idle. */
			if (scx_bpf_test_and_clear_cpu_idle(cpu)) {
				*idle = true;
				return cpu;
			}
			continue;
		}
		tier--;
		if (tier > worst) {
			worst = tier;
			victim = cpu;
			if (tier == TIER_BACKGROUND)
				break;
		}
	}
	return victim;
}

/*
 * Run a waking interactive task on @cpu next. It goes to the head of the
 * CPU's local DSQ rather than through the tier queue and a kick: a task
 * resumed in that local DSQ would otherwise run before it. The preempted
 * task resumes on @cpu afterwards.
 */
static bool preempt_cpu(struct task_struct *p, s32 prev_cpu, u64 slice, u64 enq_flags)
{
	struct cpu_ctx *cctx;
	bool idle = false;
	s32 cpu;

	cpu = preempt_target(p, prev_cpu, &idle);
	if (cpu < 0)
		return false;

	if (idle) {
		scx_bpf_dsq_insert(p, SCX_DSQ_LOCAL_ON | cpu, slice, enq_flags);
		scx_bpf_kick_cpu(cpu, SCX_KICK_IDLE);
		__sync_fetch_and_add(&nr_direct_dispatches, 1);
		return true;
	}

	cctx = try_lookup_cpu_ctx(cpu);
	if (cctx)
		cctx->resume_pid = READ_ONCE(cctx->cur_pid);
	scx_bpf_dsq_insert(p, SCX_DSQ_LOCAL_ON | cpu, slice, enq_flags | SCX_ENQ_PREEMPT);
	__sync_fetch_and_add(&nr_preempt_kicks, 1);
	return true;
}

/* Close the race with CPUs that went idle after the idle-CPU search. */
static void kick_idle_cpu(const struct task_struct *p, u32 tier, s32 prev_cpu)
{
	const struct cpumask *idle = get_idle_cpumask(prev_cpu);
	const struct cpumask *little = cast_mask(little_cpumask);
	u32 cpu;

	if (hybrid && bg_strict && tier == TIER_BACKGROUND && little) {
		cpu = bpf_cpumask_any_and_distribute(idle, little);
		if (cpu < nr_cpu_ids && !bpf_cpumask_test_cpu(cpu, p->cpus_ptr))
			cpu = nr_cpu_ids;
	} else {
		cpu = bpf_cpumask_any_and_distribute(idle, p->cpus_ptr);
	}
	scx_bpf_put_cpumask(idle);

	if (cpu < nr_cpu_ids)
		scx_bpf_kick_cpu(cpu, SCX_KICK_IDLE);
}

s32 BPF_STRUCT_OPS(aura_select_cpu, struct task_struct *p, s32 prev_cpu, u64 wake_flags)
{
	s32 cpu, this_cpu = bpf_get_smp_processor_id();
	struct task_ctx *tctx;
	u32 tier;

	if (!bpf_cpumask_test_cpu(prev_cpu, p->cpus_ptr))
		prev_cpu = bpf_cpumask_test_cpu(this_cpu, p->cpus_ptr) ?
			   this_cpu : bpf_cpumask_first(p->cpus_ptr);

	tctx = try_lookup_task_ctx(p);
	if (!tctx)
		return prev_cpu;

	if (timely_enabled)
		tctx->enq_at = scx_bpf_now();

	tier = task_update_tier(p, tctx);
	cpu = pick_idle_cpu(p, prev_cpu, wake_flags, tier);
	if (cpu >= 0) {
		scx_bpf_dsq_insert(p, SCX_DSQ_LOCAL, task_slice(p, tctx), 0);
		__sync_fetch_and_add(&nr_direct_dispatches, 1);
		return cpu;
	}

	return prev_cpu;
}

void BPF_STRUCT_OPS(aura_enqueue, struct task_struct *p, u64 enq_flags)
{
	s32 prev_cpu = scx_bpf_task_cpu(p), cpu;
	struct task_ctx *tctx;
	struct cpu_ctx *cctx;
	u64 now = scx_bpf_now(), slice;
	bool waking;
	u32 tier;

	tctx = try_lookup_task_ctx(p);
	if (!tctx) {
		scx_bpf_dsq_insert(p, tier_dsq(TIER_DEFAULT), slice_max, enq_flags);
		return;
	}

	if (timely_enabled)
		tctx->enq_at = now;
	tier = task_update_tier(p, tctx);
	slice = task_slice(p, tctx);

	/*
	 * Target claimed by ops.dispatch() at quantum expiry: an idle CPU, or
	 * a busy big core to preempt if it still runs little-core work.
	 */
	cctx = try_lookup_cpu_ctx(prev_cpu);
	if (cctx && cctx->mig_cpu > 0 && cctx->mig_pid == p->pid) {
		bool swap = cctx->mig_preempt;
		u32 cur;

		cpu = cctx->mig_cpu - 1;
		cctx->mig_cpu = 0;
		cctx->mig_preempt = 0;
		if (swap) {
			cur = cpu_cur_tier(cpu);
			if (cur && tier_prefers_little(cur - 1) &&
			    bpf_cpumask_test_cpu(cpu, p->cpus_ptr)) {
				scx_bpf_dsq_insert(p, SCX_DSQ_LOCAL_ON | cpu, slice,
						   enq_flags | SCX_ENQ_PREEMPT);
				return;
			}
		} else {
			if (bpf_cpumask_test_cpu(cpu, p->cpus_ptr)) {
				scx_bpf_dsq_insert(p, SCX_DSQ_LOCAL_ON | cpu, slice, enq_flags);
				scx_bpf_kick_cpu(cpu, SCX_KICK_IDLE);
				return;
			}
			scx_bpf_kick_cpu(cpu, SCX_KICK_IDLE);
		}
	}

	/*
	 * Resume here once the task that took the CPU is done, with the rest
	 * of the slice only: a resume must not grant extra CPU time.
	 */
	if (cctx && cctx->resume_pid && cctx->resume_pid == p->pid) {
		u64 left = time_after(cctx->slice_end, now) ? cctx->slice_end - now : 0;

		cctx->resume_pid = 0;
		if (!(enq_flags & SCX_ENQ_REENQ) && scx_bpf_task_running(p) && left) {
			scx_bpf_dsq_insert(p, SCX_DSQ_LOCAL, MIN(left, slice_max), enq_flags);
			__sync_fetch_and_add(&nr_local_dispatches, 1);
			return;
		}
	}

	waking = !__COMPAT_is_enq_cpu_selected(enq_flags) && !scx_bpf_task_running(p);

	/* The CPU was taken by a higher scheduling class: try another one. */
	if (enq_flags & SCX_ENQ_REENQ) {
		__sync_fetch_and_add(&nr_reenq, 1);
		if (!is_pcpu_task(p) && try_direct(p, prev_cpu, tier, slice, enq_flags))
			return;
		goto queue;
	}

	/*
	 * ops.dispatch() let @p's slice lapse without claiming a target, so
	 * its tier cannot stay on this CPU (strict background on a big core)
	 * or the CPU is throttled. Otherwise keep it here and make sure the
	 * CPU reschedules, since nothing else will.
	 */
	if (enq_flags & SCX_ENQ_LAST) {
		if (!is_throttled() && (tier_eligible(prev_cpu) & (1U << tier))) {
			scx_bpf_dsq_insert(p, SCX_DSQ_LOCAL, slice, enq_flags);
			scx_bpf_kick_cpu(prev_cpu, SCX_KICK_IDLE);
			return;
		}
		goto queue;
	}

	if (is_task_sticky(tctx) ||
	    (local_kthreads && is_kthread(p) && p->nr_cpus_allowed == 1) ||
	    (local_pcpu && is_pcpu_task(p))) {
		scx_bpf_dsq_insert(p, SCX_DSQ_LOCAL, slice, enq_flags);
		__sync_fetch_and_add(&nr_local_dispatches, 1);
		return;
	}

	if (waking && !is_pcpu_task(p) && try_direct(p, prev_cpu, tier, slice, enq_flags))
		return;

queue:
	if (tier == TIER_INTERACTIVE && !scx_bpf_task_running(p) && !is_pcpu_task(p) &&
	    preempt_cpu(p, prev_cpu, slice, enq_flags))
		return;

	scx_bpf_dsq_insert_vtime(p, tier_dsq(tier), slice, task_dl(p, tctx, tier), enq_flags);
	__sync_fetch_and_add(&nr_tier_enqueues[tier_idx(tier)], 1);
	tier_mark_runnable(tier, now);

	/*
	 * Waking, re-enqueued and preempted tasks alike: an idle CPU that
	 * could run @p will not look at the tier queues until kicked.
	 */
	kick_idle_cpu(p, tier, prev_cpu);
}

static __always_inline bool consume_tier(u32 tier, u64 now)
{
	tier = tier_idx(tier);
	if (!scx_bpf_dsq_move_to_local(tier_dsq(tier), 0))
		return false;

	__sync_fetch_and_add(&nr_tier_dispatches[tier], 1);
	if (!scx_bpf_dsq_nr_queued(tier_dsq(tier))) {
		bpf_spin_lock(&root_lock);
		if (root_bkt[tier].runnable)
			rb_empty(tier, now);
		bpf_spin_unlock(&root_lock);
	}
	return true;
}

/*
 * Charge the runtime since the last accounting point to @p and its group.
 * wakeup_freq only changes on wakeups, so it is also bounded by the rate
 * implied by the CPU time used since the last one: a task that stops
 * sleeping loses its wakeup credit instead of keeping it forever.
 */
static void task_charge(struct task_struct *p, struct task_ctx *tctx, u64 now)
{
	u64 ran = 0, delta;

	if (tctx->last_run_at && now > tctx->last_run_at)
		ran = now - tctx->last_run_at;
	tctx->last_run_at = now;
	if (!ran)
		return;

	tctx->burst += ran;
	if (tctx->wakeup_freq && tctx->burst > NSEC_PER_MSEC)
		tctx->wakeup_freq = MIN(tctx->wakeup_freq,
					(100ULL * NSEC_PER_MSEC) / tctx->burst);

	delta = scale_by_task_weight_inverse(p, ran);
	scx_bpf_task_set_dsq_vtime(p, p->scx.dsq_vtime + delta);
	tctx->awake_vtime += delta;
	group_charge(p, ran);
}

static void keep_prev(struct task_struct *p, struct task_ctx *tctx, s32 cpu,
		      struct cpu_ctx *cctx, u64 now)
{
	u32 tier = tctx->tier & TIER_MASK;
	u64 slice = task_slice(p, tctx);

	scx_bpf_task_set_slice(p, slice);
	cctx->slice_end = now + slice;
	set_cur_tier(cctx, tier + 1);
	util_account(cctx, now);
	perf_update(cpu, cctx, tier, now);
	__sync_fetch_and_add(&nr_keep_running, 1);
}

/*
 * Within its tier, @prev competes with the queue head on deadline, with one
 * slice of grace: swapping two tasks over a marginal deadline difference
 * migrates both of them.
 */
static bool prev_before_head(struct task_struct *prev, struct task_ctx *tctx,
			     u32 tier, u64 now)
{
	struct task_struct *head = __COMPAT_scx_bpf_dsq_peek(tier_dsq(tier));

	/* @prev's runtime was charged by ops.dispatch() just before. */
	if (!head)
		return true;
	return time_before(prev->scx.dsq_vtime, head->scx.dsq_vtime +
			   scale_by_task_weight_inverse(prev, slice_max));
}

static u32 tiers_queued(void)
{
	u32 mask = 0, t;

#pragma unroll
	for (t = 0; t < NR_TIERS; t++)
		if (scx_bpf_dsq_nr_queued(tier_dsq(t)) > 0)
			mask |= 1U << t;
	return mask;
}

void BPF_STRUCT_OPS(aura_dispatch, s32 cpu, struct task_struct *prev)
{
	struct task_ctx *ptctx = NULL;
	struct cpu_ctx *cctx;
	u32 qmask, elig, cand, how = SEL_NATURAL, t;
	s32 tier, prev_tier = -1;
	u64 now;

	if (is_throttled())
		return;

	cctx = try_lookup_cpu_ctx(cpu);
	if (!cctx)
		return;
	now = scx_bpf_now();

	release_stale_claim(cctx);
	cctx->resume_pid = 0;

	qmask = tiers_queued();
	elig = tier_eligible(cpu);
	/*
	 * The tier queues are shared by both core classes. A big core runs
	 * background work only when nothing else waits, so background work
	 * does not take big cores while higher tiers queue up on little ones.
	 */
	if (hybrid && !cpu_is_little(cpu) && (qmask & ((1U << TIER_BACKGROUND) - 1)))
		elig &= ~(1U << TIER_BACKGROUND);
	cand = qmask & elig;

	if (prev && is_task_queued(prev)) {
		ptctx = try_lookup_task_ctx(prev);
		if (ptctx) {
			task_charge(prev, ptctx, now);
			task_update_tier(prev, ptctx);
			if (prev_stays(prev, ptctx, cpu, cctx, elig)) {
				prev_tier = ptctx->tier & TIER_MASK;
				cand |= 1U << prev_tier;
			}
		}
	}
	if (!cand)
		return;

	bpf_spin_lock(&root_lock);
	rb_reconcile(qmask, now);
	tier = rb_select(cand, now, &how);
	bpf_spin_unlock(&root_lock);

	switch (how) {
	case SEL_WARP:
		__sync_fetch_and_add(&nr_sel_warp, 1);
		break;
	case SEL_STARVE_OPEN:
		__sync_fetch_and_add(&nr_sel_starve_open, 1);
		dbg_msg("starvation window: tier %d cpu %d", tier, cpu);
		break;
	case SEL_STARVE:
		__sync_fetch_and_add(&nr_sel_starve, 1);
		break;
	default:
		__sync_fetch_and_add(&nr_sel_natural, 1);
		break;
	}

	if (tier < 0)
		return;

	if (tier == prev_tier && ptctx &&
	    (!(qmask & (1U << tier)) || prev_before_head(prev, ptctx, tier, now))) {
		keep_prev(prev, ptctx, cpu, cctx, now);
		return;
	}
	if ((qmask & (1U << tier)) && consume_tier(tier, now)) {
		/*
		 * A lower tier got an EDF or starvation turn: @prev keeps its
		 * place and resumes here right after it. When a higher tier
		 * wins, @prev goes back to its tier queue and competes again.
		 */
		if (ptctx && prev_tier >= 0 && tier > prev_tier) {
			cctx->resume_pid = prev->pid;
			cctx->slice_end = now + task_slice(prev, ptctx);
		}
		return;
	}
	if (tier == prev_tier && ptctx) {
		keep_prev(prev, ptctx, cpu, cctx, now);
		return;
	}

	/*
	 * Nothing in the selected tier can run here: fall back by priority,
	 * without passing over @prev for a lower tier.
	 */
#pragma unroll
	for (t = 0; t < NR_TIERS; t++) {
		if ((s32)t == prev_tier && ptctx) {
			keep_prev(prev, ptctx, cpu, cctx, now);
			return;
		}
		if ((s32)t != tier && (cand & qmask & (1U << t)) && consume_tier(t, now))
			return;
	}
}

void BPF_STRUCT_OPS(aura_running, struct task_struct *p)
{
	s32 cpu = scx_bpf_task_cpu(p);
	struct task_ctx *tctx;
	struct cpu_ctx *cctx;
	u64 now = scx_bpf_now();
	u32 tier;

	__sync_fetch_and_add(&nr_running, 1);

	tctx = try_lookup_task_ctx(p);
	cctx = try_lookup_cpu_ctx(cpu);
	if (!tctx || !cctx)
		return;

	tier = tier_idx(tctx->tier);
	tctx->last_run_at = now;
	if (time_before(vtime_now[tier], p->scx.dsq_vtime))
		vtime_now[tier] = p->scx.dsq_vtime;

	if (timely_enabled)
		timely_sample(tctx, now);

	set_cur_tier(cctx, tier + 1);
	WRITE_ONCE(cctx->cur_pid, p->pid);
	cctx->slice_end = now + p->scx.slice;
	cctx->run_start = now;
	perf_update(cpu, cctx, tier, now);
}

void BPF_STRUCT_OPS(aura_stopping, struct task_struct *p, bool runnable)
{
	s32 cpu = scx_bpf_task_cpu(p);
	struct task_ctx *tctx;
	struct cpu_ctx *cctx;
	u64 now = scx_bpf_now();

	__sync_fetch_and_sub(&nr_running, 1);

	tctx = try_lookup_task_ctx(p);
	if (!tctx)
		return;

	task_charge(p, tctx, now);

	cctx = try_lookup_cpu_ctx(cpu);
	if (!cctx)
		return;
	util_account(cctx, now);
	cctx->run_start = 0;
	set_cur_tier(cctx, 0);
	WRITE_ONCE(cctx->cur_pid, 0);
}

void BPF_STRUCT_OPS(aura_runnable, struct task_struct *p, u64 enq_flags)
{
	u64 now = scx_bpf_now(), delta_t;
	struct task_ctx *tctx;

	group_runnable(p, now);

	if (!(enq_flags & SCX_ENQ_WAKEUP))
		return;
	tctx = try_lookup_task_ctx(p);
	if (!tctx)
		return;

	tctx->awake_vtime = 0;
	tctx->avg_burst = calc_avg(tctx->avg_burst, tctx->burst);
	tctx->burst = 0;
	delta_t = now > tctx->last_woke_at ? now - tctx->last_woke_at : 1;
	tctx->wakeup_freq = MIN(update_freq(tctx->wakeup_freq, delta_t), MAX_WAKEUP_FREQ);
	tctx->last_woke_at = now;
}

void BPF_STRUCT_OPS(aura_quiescent, struct task_struct *p, u64 deq_flags)
{
	group_quiescent(p, scx_bpf_now());
}

/*
 * When a higher scheduling class takes a CPU, hand the tasks waiting in its
 * local DSQ back to ops.enqueue() (with SCX_ENQ_REENQ) so they can run
 * elsewhere. ops.cpu_release() is deprecated on kernels that can re-enqueue
 * from any context; user space switches to the sched_switch hook there.
 */
void BPF_STRUCT_OPS(aura_cpu_release, s32 cpu, struct scx_cpu_release_args *args)
{
	scx_bpf_reenqueue_local();
}

SEC("?tp_btf/sched_switch")
int BPF_PROG(aura_sched_switch, bool preempt, struct task_struct *prev,
	     struct task_struct *next, unsigned int prev_state)
{
	/* Only loaded when the any-context kfunc exists (see main.rs). */
	if (next->prio < MAX_RT_PRIO && prev->prio >= MAX_RT_PRIO &&
	    __COMPAT_scx_bpf_reenqueue_local_from_anywhere())
		scx_bpf_reenqueue_local___v2___compat();
	return 0;
}

void BPF_STRUCT_OPS(aura_enable, struct task_struct *p)
{
	scx_bpf_task_set_dsq_vtime(p, vtime_now[TIER_DEFAULT]);
}

s32 BPF_STRUCT_OPS(aura_init_task, struct task_struct *p, struct scx_init_task_args *args)
{
	struct task_ctx *tctx;

	tctx = bpf_task_storage_get(&task_ctx_stor, p, 0, BPF_LOCAL_STORAGE_GET_F_CREATE);
	if (!tctx)
		return -ENOMEM;

	/*
	 * No burst history yet: assume one full slice, so a new task is
	 * neither interactive nor sticky until its wakeups show short bursts.
	 */
	tctx->tier = TIER_DEFAULT;
	tctx->avg_burst = slice_max;
	tctx->gain = FP_ONE;
	return 0;
}

static s32 get_nr_online_cpus(void)
{
	const struct cpumask *online = scx_bpf_get_online_cpumask();
	int n = bpf_cpumask_weight(online);

	scx_bpf_put_cpumask(online);
	return n;
}

static int calloc_cpumask(struct bpf_cpumask **p_cpumask)
{
	struct bpf_cpumask *cpumask = bpf_cpumask_create();

	if (!cpumask)
		return -ENOMEM;
	cpumask = bpf_kptr_xchg(p_cpumask, cpumask);
	if (cpumask)
		bpf_cpumask_release(cpumask);
	return 0;
}

static int init_cpumask(struct bpf_cpumask **cpumask)
{
	struct bpf_cpumask *mask = *cpumask;
	int err;

	if (mask)
		return 0;
	err = calloc_cpumask(cpumask);
	if (!err)
		mask = *cpumask;
	if (!mask)
		err = -ENOMEM;
	return err;
}

static int init_cpu_classes(void)
{
	struct bpf_cpumask *big, *little;
	int err;
	u32 i;

	err = init_cpumask(&big_cpumask);
	if (!err)
		err = init_cpumask(&little_cpumask);
	if (err)
		return err;

	bpf_rcu_read_lock();
	big = big_cpumask;
	little = little_cpumask;
	if (big && little) {
		bpf_for(i, 0, nr_preferred) {
			s32 cpu;

			if (i >= MAX_CPUS)
				break;
			cpu = preferred_cpus[cpu_idx(i)];
			if (cpu_is_little(cpu))
				bpf_cpumask_set_cpu(cpu, little);
			else
				bpf_cpumask_set_cpu(cpu, big);
		}
	}
	bpf_rcu_read_unlock();

	return 0;
}

static void init_tiers(void)
{
	/* The fixed-size tables are indexed with constants so they fold away. */
	const u64 wcel_us[NR_TIERS] = { 0, 75000, 150000, 250000 };
	const u64 warp_us[NR_TIERS] = { 8000, 2000, 1000, 0 };
	const u64 quantum_us[NR_TIERS] = { 10000, 10000, 4000, 2000 };
	u32 t;

#pragma unroll
	for (t = 0; t < NR_TIERS; t++) {
		tier_wcel_ns[t] = wcel_us[t] * slice_max / XNU_REF_QUANTUM_US;
		tier_warp_ns[t] = warp_enabled ? warp_us[t] * slice_max / XNU_REF_QUANTUM_US : 0;
		tier_quantum_ns[t] = quantum_us[t] * slice_max / XNU_REF_QUANTUM_US;

		root_bkt[t].deadline = 0;
		root_bkt[t].warp_remaining = tier_warp_ns[t];
		root_bkt[t].warped_deadline = WARP_UNUSED;
		root_bkt[t].starvation_ts = 0;
		root_bkt[t].runnable = 0;
		root_bkt[t].warp_avail = 0;
		root_bkt[t].starving = 0;
	}
}

static void init_cpuperf_target(void)
{
	const struct cpumask *online;
	s32 cpu;

	if (cpufreq_perf_lvl < 0)
		return;

	online = scx_bpf_get_online_cpumask();
	bpf_for(cpu, 0, nr_cpu_ids) {
		if (bpf_cpumask_test_cpu(cpu, online))
			scx_bpf_cpuperf_set(cpu, MIN((u64)cpufreq_perf_lvl, SCX_CPUPERF_ONE));
	}
	scx_bpf_put_cpumask(online);
}

SEC("syscall")
int enable_sibling_cpu(struct domain_arg *input)
{
	struct cpu_ctx *cctx;
	struct bpf_cpumask *mask, **pmask;
	int err;

	cctx = try_lookup_cpu_ctx(input->cpu_id);
	if (!cctx)
		return -ENOENT;

	pmask = &cctx->smt;
	err = init_cpumask(pmask);
	if (err)
		return err;

	bpf_rcu_read_lock();
	mask = *pmask;
	if (mask)
		bpf_cpumask_set_cpu(input->sibling_cpu_id, mask);
	bpf_rcu_read_unlock();

	return 0;
}

SEC("syscall")
int enable_primary_cpu(struct cpu_arg *input)
{
	struct bpf_cpumask *mask;
	int err;

	err = init_cpumask(&primary_cpumask);
	if (err)
		return err;

	bpf_rcu_read_lock();
	mask = primary_cpumask;
	if (mask) {
		s32 cpu = input->cpu_id;

		if (cpu < 0)
			bpf_cpumask_clear(mask);
		else
			bpf_cpumask_set_cpu(cpu, mask);
	}
	bpf_rcu_read_unlock();

	return 0;
}

static int throttle_timerfn(void *map, int *key, struct bpf_timer *timer)
{
	bool throttled = is_throttled();
	u64 flags = throttled ? SCX_KICK_IDLE : SCX_KICK_PREEMPT;
	u64 duration = throttled ? slice_max : throttle_ns;
	s32 cpu;
	int err;

	set_throttled(!throttled);
	bpf_for(cpu, 0, nr_cpu_ids)
		scx_bpf_kick_cpu(cpu, flags);

	err = bpf_timer_start(timer, duration, 0);
	if (err)
		scx_bpf_error("Failed to re-arm throttle timer");

	return 0;
}

s32 BPF_STRUCT_OPS_SLEEPABLE(aura_init)
{
	struct bpf_timer *timer;
	u32 key = 0;
	int err, i;

	nr_online_cpus = get_nr_online_cpus();
	nr_cpu_ids = scx_bpf_nr_cpu_ids();

	init_tiers();
	init_cpuperf_target();

	bpf_for(i, 0, NR_TIERS) {
		err = scx_bpf_create_dsq(tier_dsq(i), -1);
		if (err) {
			scx_bpf_error("failed to create tier DSQ %d: %d", i, err);
			return err;
		}
	}

	err = init_cpumask(&primary_cpumask);
	if (err)
		return err;
	err = init_cpu_classes();
	if (err)
		return err;

	timer = bpf_map_lookup_elem(&throttle_timer, &key);
	if (!timer) {
		scx_bpf_error("Failed to lookup throttle timer");
		return -ESRCH;
	}

	if (throttle_ns) {
		bpf_timer_init(timer, &throttle_timer, CLOCK_BOOTTIME);
		bpf_timer_set_callback(timer, throttle_timerfn);
		err = bpf_timer_start(timer, slice_max, 0);
		if (err) {
			scx_bpf_error("Failed to arm throttle timer");
			return err;
		}
	}

	return 0;
}

void BPF_STRUCT_OPS(aura_exit, struct scx_exit_info *ei)
{
	UEI_RECORD(uei, ei);
}

SCX_OPS_DEFINE(aura_ops,
	       .select_cpu		= (void *)aura_select_cpu,
	       .enqueue			= (void *)aura_enqueue,
	       .dispatch		= (void *)aura_dispatch,
	       .running			= (void *)aura_running,
	       .stopping		= (void *)aura_stopping,
	       .runnable		= (void *)aura_runnable,
	       .quiescent		= (void *)aura_quiescent,
	       .cpu_release		= (void *)aura_cpu_release,
	       .enable			= (void *)aura_enable,
	       .init_task		= (void *)aura_init_task,
	       .init			= (void *)aura_init,
	       .exit			= (void *)aura_exit,
	       .timeout_ms		= STARVATION_MS,
	       .name			= "aura");
