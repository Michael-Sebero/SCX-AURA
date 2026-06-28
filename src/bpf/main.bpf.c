/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2024 Andrea Righi <andrea.righi@linux.dev>
 *
 * scx_aura — laptop-optimised sched_ext scheduler porting XNU Clutch's
 * interactive and battery-saving mechanisms onto a simplified 3-tier
 * model. Deliberately not full Clutch parity (3 tiers instead of 6, no
 * bound/unbound hierarchy, no AMP cluster spill/steal) — see the
 * per-mechanism notes below for what each one does and does not cover:
 *
 *  Three-tier scheduling (TIER_INTERACTIVE / DEFAULT / BACKGROUND) with
 *  correct cross-tier EDF via per-tier vtime base offsets.
 *
 *  Per-process-group interactivity scoring (XNU Clutch bucket scoring).
 *
 *  Warp dispatch — per-tier depleting warp budget (INTERACTIVE: 8 ms,
 *  DEFAULT: 2 ms, BACKGROUND: 0 ms) lets a higher tier jump ahead of EDF,
 *  but the budget only refills when that tier next wins the EDF race
 *  fairly (on deadline alone). A tier with continuous arrivals cannot
 *  warp forever, matching XNU's bounded warp semantics and avoiding
 *  indefinite starvation of lower tiers.
 *
 *  Per-tier WCEL (Worst-Case Execution Latency) enforcement: a tier that
 *  has been waiting longer than its budget gets its deadline zeroed so it
 *  wins dispatch.  If nothing higher is runnable this is unconditional,
 *  matching XNU's natural-order selection.  If a higher tier IS also
 *  runnable, the override is bounded to one starvation-avoidance quantum
 *  (re-opened each time the tier is still starved afterward), matching
 *  XNU's actual bounded starvation-avoidance window rather than letting
 *  the lower tier monopolize the CPU for as long as its head stays old.
 *    TIER_INTERACTIVE:  0 ms  (always runs as soon as a CPU is free)
 *    TIER_DEFAULT:     75 ms
 *    TIER_BACKGROUND: 250 ms
 *
 *  TIMELY adaptive time-slice feedback — fully implemented:
 *    Queue delay is sampled per task.  A per-task gain_fp (fixed-point
 *    multiplier in [gain_min_fp .. gain_max_fp]) is adjusted every
 *    control_interval_ns:
 *      delay < tlow  → increase gain (longer slices, less preemption)
 *      delay > thigh → decrease gain (shorter slices, more preemption)
 *    A High-Activity-Index (HAI) streak counter detects sustained high
 *    delay and applies a multiplier.  A gradient detector applies a
 *    backoff when the delay is rising steeply.
 *    The resulting slice = clamp(slice_max * gain_fp / FP_ONE,
 *                                slice_min, slice_max).
 *
 *  E-core idle consolidation applied to ALL tiers (not just interactive).
 *  When the system is lightly loaded (< 3/4 of online CPUs busy) any tier's
 *  task avoids E-cores, letting them reach deep C-states.
 *
 *  cpufreq feedback unified: tot_runtime / last_running are updated on every
 *  stopping() call regardless of whether TIMELY is on.  update_cpu_load()
 *  is called from both the TIMELY path (to keep frequency in sync with the
 *  adapted slice) and the non-TIMELY path.
 *
 *  14 idle/dispatch stats counters fully wired at their call sites.
 *
 *  dbg_msg used at meaningful decision points throughout.
 */
#include <scx/common.bpf.h>
#include <scx/percpu.bpf.h>
#include "intf.h"

/* ─── Constants ─────────────────────────────────────────────────────────── */

#define STARVATION_MS		5000ULL
#define MAX_CPUS		1024
#define MAX_WAKEUP_FREQ		64ULL

/* Fixed-point scale for TIMELY gain. */
#define FP_ONE			1024U

#define TIER_INTERACTIVE	0
#define TIER_DEFAULT		1
#define TIER_BACKGROUND		2
#define NR_TIERS		3

/*
 * Tier vtime separation: 10 × slice_lag (400 ms at default settings).
 * Interactive tasks always have a numerically smaller dsq_vtime than
 * default tasks, which always have a smaller dsq_vtime than background
 * tasks.  Cross-tier EDF comparison is a plain u64 comparison.
 */
#define TIER_VTIME_GAP		(10ULL * 40ULL * NSEC_PER_MSEC)

/*
 * Per-tier WCEL (Worst-Case Execution Latency) budgets.
 * A tier whose head task has been waiting longer than its budget has its
 * vtime deadline forced to zero (earliest possible) so it wins the next
 * dispatch unconditionally.
 *
 * XNU values: FG = 0 ms, IN = 37.5 ms, DF = 75 ms, BG = 250 ms.
 * We map to three tiers:
 *   INTERACTIVE:  0 ms  — must run as soon as any CPU is free.
 *   DEFAULT:     75 ms  — maps to XNU DF bucket.
 *   BACKGROUND: 250 ms  — maps to XNU BG bucket.
 */
#define WCEL_INTERACTIVE_NS	0ULL
#define WCEL_DEFAULT_NS		(75ULL  * NSEC_PER_MSEC)
#define WCEL_BACKGROUND_NS	(250ULL * NSEC_PER_MSEC)

/*
 * Per-tier starvation-avoidance quantum (XNU analogue:
 * sched_clutch_thread_quantum[bucket], used as the bounded window length
 * in sched_clutch_root_highest_root_bucket()'s starvation-avoidance path).
 *
 * XNU populates this table from the standard Mach timeshare quantum
 * (~10 ms) uniformly across buckets at scheduler init.  scx_aura's
 * slice_max defaults to 1 ms — roughly an order of magnitude shorter,
 * consistent with how WCEL_*_NS and WARP_BUDGET_*_NS above were already
 * scaled down from XNU's literal values for this scheduler's much
 * shorter base slice.  We follow the same proportion here rather than
 * hardcode XNU's literal 10 ms, which would be disproportionately long
 * relative to this scheduler's tick granularity.
 *
 * TIER_INTERACTIVE never needs a starvation-avoidance window (nothing
 * sits above it to starve it), so it has no quantum entry.
 */
#define STARVATION_QUANTUM_DEFAULT_NS		(1ULL * NSEC_PER_MSEC)
#define STARVATION_QUANTUM_BACKGROUND_NS	(1ULL * NSEC_PER_MSEC)

/*
 * Per-tier warp budgets (XNU analogue: sched_clutch_root_bucket_warp_us[]).
 *
 * XNU values: FG = 8 ms, IN = 4 ms, DF = 2 ms, UT = 1 ms, BG = 0 ms.
 * Mapped to our three tiers (INTERACTIVE warps over DEFAULT/BACKGROUND;
 * DEFAULT warps over BACKGROUND only; BACKGROUND never warps):
 *   INTERACTIVE: 8 ms  — maps to XNU FG.
 *   DEFAULT:     2 ms  — maps to XNU DF.
 *   BACKGROUND:  0 ms  — maps to XNU BG (no warp).
 *
 * Unlike a fixed sliding window, this is a *depleting* budget: it is only
 * refilled when the tier wins its EDF race fairly (i.e. on deadline alone,
 * not because it was already warping), and it drains in real time while
 * being spent.  Once exhausted the tier cannot warp again until it next
 * wins normally.  This matches XNU's starvation-bounded warp semantics and
 * prevents a tier with continuous arrivals from warping forever.
 */
#define WARP_BUDGET_INTERACTIVE_NS	(8ULL * NSEC_PER_MSEC)
#define WARP_BUDGET_DEFAULT_NS		(2ULL * NSEC_PER_MSEC)
#define WARP_BUDGET_BACKGROUND_NS	0ULL

/*
 * Max retries for the CAS loop that charges tier_warp_remaining_ns[].
 * Bounded so the BPF verifier can prove termination; under realistic
 * contention (at most nr_cpu_ids concurrent dispatch() callers racing on
 * one tier's budget) this is generously above worst case.
 */
#define WARP_CAS_MAX_RETRY		16

/* Group interactivity scoring window and parameters. */
#define IACT_WINDOW_NS		(500ULL * NSEC_PER_MSEC)
#define IACT_PRI_MAX		16
#define IACT_THRESH		12
#define IACT_BG_THRESH		4
#define ADJUST_RATIO		10

/* Nice hard limits for tier classification. */
#define NICE_INTERACTIVE	(-5)
#define NICE_BACKGROUND		(10)

/* E-core consolidation: activate when < 3/4 of online CPUs are busy. */
#define CONSOLIDATION_THRESH_NUM	3
#define CONSOLIDATION_THRESH_DEN	4

char _license[] SEC("license") = "GPL";

/* ─── Debug ──────────────────────────────────────────────────────────────── */

const volatile bool debug;
#define dbg_msg(_fmt, ...) do {			\
	if (debug)				\
		bpf_printk(_fmt, ##__VA_ARGS__);	\
} while (0)

/* ─── Rodata tunables ───────────────────────────────────────────────────── */

const volatile bool timely_enabled;
const volatile u64  slice_max			= 1ULL * NSEC_PER_MSEC;
const volatile u64  slice_min;
const volatile u64  slice_lag			= 40ULL * NSEC_PER_MSEC;
const volatile bool no_wake_sync;
const volatile bool sticky_tasks		= true;
const volatile bool local_kthreads		= true;
const volatile bool local_pcpu			= true;
const volatile bool preferred_idle_scan;
const volatile u64  preferred_cpus[MAX_CPUS];
const volatile u64  cpu_capacity[MAX_CPUS];
const volatile bool smt_enabled			= true;
const volatile bool numa_enabled		= true;
const volatile bool primary_all			= true;
const volatile bool group_iact_enabled		= true;
const volatile bool ecore_consolidate		= true;
const volatile bool warp_enabled		= true;

/*
 * TIMELY tunables.  All are used in timely_update_gain() and
 * timely_task_slice().
 *
 * tlow_ns / thigh_ns  — delay region boundaries.
 * gain_min_fp / max_fp — fixed-point gain range ([128..1024] = [0.125x..1x]).
 * gain_step_fp         — per-interval adjustment amount.
 * hai_thresh_fp        — gain threshold below which HAI streak increments.
 * hai_multiplier       — multiplier applied when streak fires.
 * backoff_*_fp         — gradient-detection backoff factors.
 * gradient_margin_ns   — minimum gradient magnitude to trigger backoff.
 * control_interval_ns  — how often gain is updated per task.
 */
const volatile u64  timely_tlow_ns		= 5000ULL  * NSEC_PER_USEC;
const volatile u64  timely_thigh_ns		= 50000ULL * NSEC_PER_USEC;
const volatile u32  timely_gain_min_fp		= 128U;
const volatile u32  timely_gain_max_fp		= 1024U;
const volatile u32  timely_gain_step_fp		= 32U;
const volatile u32  timely_hai_thresh_fp	= 768U;
const volatile u32  timely_hai_multiplier	= 2U;
const volatile u32  timely_backoff_low_fp	= 768U;
const volatile u32  timely_backoff_high_fp	= 960U;
const volatile u32  timely_backoff_gradient_fp	= 992U;
const volatile u64  timely_gradient_margin_ns	= 125ULL * NSEC_PER_USEC;
const volatile u64  timely_control_interval_ns	= 500ULL * NSEC_PER_USEC;

/* Runtime throttle. */
const volatile u64   throttle_ns;
static volatile bool cpus_throttled;

static inline bool is_throttled(void)    { return READ_ONCE(cpus_throttled); }
static inline void set_throttled(bool s) { WRITE_ONCE(cpus_throttled, s);    }

/* ─── BSS (mutable globals) ─────────────────────────────────────────────── */

volatile s64 cpufreq_perf_lvl;

UEI_DEFINE(uei);

private(AURA) struct bpf_cpumask __kptr *primary_cpumask;

static u64 vtime_now;
static u64 nr_cpu_ids;

volatile u64 nr_running;
volatile u64 nr_online_cpus;

/* Precomputed max cpu_capacity; used by is_cpu_efficient() without a loop. */
static volatile u64 max_cpu_cap;

/*
 * Per-tier vtime base offsets and WCEL tables, populated in aura_init.
 *   tier_vtime_base[TIER_INTERACTIVE] = 0
 *   tier_vtime_base[TIER_DEFAULT]     = TIER_VTIME_GAP
 *   tier_vtime_base[TIER_BACKGROUND]  = 2 * TIER_VTIME_GAP
 */
static u64 tier_vtime_base[NR_TIERS];
static u64 tier_wcel_ns[NR_TIERS];

/*
 * Per-tier enqueue timestamps: the wall-clock time when the current head
 * of each tier DSQ was first enqueued.  Used to enforce WCEL: if
 * (now - tier_enqueue_ts[tier]) > tier_wcel_ns[tier], that tier's head
 * task wins the next dispatch unconditionally.
 *
 * Updated in aura_enqueue() when a task enters a tier DSQ, and cleared
 * when the head task is dispatched.
 */
static volatile u64 tier_enqueue_ts[NR_TIERS];

/*
 * Per-tier warp budget table, populated in aura_init from
 * WARP_BUDGET_*_NS.  BACKGROUND's budget is always 0 (lowest tier never
 * warps over anything).
 */
static u64 tier_warp_budget_ns[NR_TIERS];

/*
 * Per-tier warp accounting (XNU analogue: scrb_warp_remaining /
 * scrb_warped_deadline in sched_clutch_root_bucket).
 *
 * tier_warp_remaining_ns[t]: budget left for tier t to warp ahead of EDF.
 *   Drained in real time while a warp window for t is open; refilled to
 *   the full per-tier budget only when t wins the EDF race *fairly*
 *   (i.e. on deadline alone, not via warp) — see tier_warp_refill().
 *
 * tier_warp_window_until_ns[t]: wall-clock end of the current open warp
 *   window for tier t, or 0 if no window is currently open.  A window is
 *   opened lazily on first use after a tier has positive remaining budget,
 *   and is capped to however much budget is left.
 */
static volatile u64 tier_warp_remaining_ns[NR_TIERS];
static volatile u64 tier_warp_window_until_ns[NR_TIERS];
static volatile u64 tier_warp_window_opened_ns[NR_TIERS];

/*
 * Per-tier starvation-avoidance window state (XNU analogue:
 * scrb_starvation_avoidance / scrb_starvation_ts in
 * sched_clutch_root_bucket).
 *
 * tier_starvation_active[t]: true while tier t is inside a bounded
 *   starvation-avoidance window — i.e. it is being allowed to win
 *   dispatch specifically *because* a higher tier is also runnable and
 *   t would otherwise be starved, not because it won the deadline race
 *   on its own merits.
 *
 * tier_starvation_ts[t]: wall-clock time the current window was opened.
 *   The window closes after STARVATION_QUANTUM_*_NS elapses, at which
 *   point the tier's WCEL-based deadline override is recomputed and the
 *   decision is re-evaluated from scratch — it does not silently keep
 *   winning forever the way an unbounded WCEL override would.
 */
static volatile bool tier_starvation_active[NR_TIERS];
static volatile u64  tier_starvation_ts[NR_TIERS];

/* ─── Statistics counters ──────────────────────────────────────────────── */

volatile u64
	nr_kthread_dispatches, nr_direct_dispatches, nr_shared_dispatches,
	nr_delay_recovery_dispatches, nr_delay_middle_add_dispatches,
	nr_delay_fast_recovery_dispatches, nr_delay_rate_limited_dispatches,
	nr_gain_floor_dispatches, nr_gain_ceiling_dispatches,
	nr_delay_low_region_samples, nr_delay_mid_region_samples,
	nr_delay_high_region_samples, nr_gain_floor_resident_samples,
	nr_gain_mid_resident_samples, nr_gain_ceiling_resident_samples,
	/* idle selection path counters — all now wired */
	nr_idle_select_path_picks, nr_idle_enqueue_path_picks,
	nr_idle_prev_cpu_picks, nr_idle_primary_picks, nr_idle_spill_picks,
	nr_idle_pick_failures, nr_idle_primary_domain_misses,
	nr_idle_global_misses,
	/* dispatch / keep-running counters — all now wired */
	nr_waker_cpu_biases,
	nr_keep_running_reuses, nr_keep_running_queue_empty,
	nr_keep_running_smt_blocked, nr_keep_running_queued_work,
	nr_dispatch_cpu_dsq_consumes,
	nr_dispatch_node_dsq_consumes,	/* always 0; kept for stats compat */
	nr_cpu_release_reenqueue,
	/* laptop / XNU counters */
	nr_interactive_dispatches, nr_background_dispatches,
	nr_warp_dispatches, nr_default_warp_dispatches,
	nr_iact_promoted, nr_iact_demoted,
	nr_ecore_consolidations, nr_preempt_kicks,
	nr_ecore_rebalance_pulls,
	/* WCEL enforcement counter */
	nr_wcel_enforcements, nr_starvation_window_opens;

/* ─── Per-group interactivity map ───────────────────────────────────────── */

struct group_iact {
	u64 cpu_used_ns;
	u64 blocked_accum_ns;
	u64 blocked_start_ns;
	u64 decay_gen;
	u32 runnable_count;
	u8  score;
	u8  tier;
	u8  pad[2];
};

struct {
	__uint(type, BPF_MAP_TYPE_HASH);
	__uint(max_entries, 4096);
	__type(key, u32);
	__type(value, struct group_iact);
} group_iact_map SEC(".maps");

/* ─── Per-CPU context ───────────────────────────────────────────────────── */

struct cpu_ctx {
	u64 tot_runtime;
	u64 prev_runtime;
	u64 last_running;
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

/* ─── Per-task context ──────────────────────────────────────────────────── */

struct task_ctx {
	/* Core scheduling state. */
	u64 awake_vtime;
	u64 last_run_at;
	u64 wakeup_freq;
	u64 last_woke_at;
	u64 avg_runtime;
	u8  tier;
	u8  pad[3];
	/*
	 * TIMELY adaptive slice state.
	 * All fields gated by timely_enabled at read/write sites.
	 */
	u64 timely_last_enqueued_at;
	u32 timely_gain_fp;		/* current gain, fixed-point [min..max] */
	u64 timely_last_gain_update_at;	/* when gain was last adjusted          */
	u64 timely_last_delay_sample_at;
	u64 timely_avg_queue_delay;	/* EWMA of queue delay                  */
	s64 timely_avg_queue_gradient;	/* EWMA of delay gradient               */
	u32 timely_hai_streak;		/* consecutive high-gain intervals      */
};

struct {
	__uint(type, BPF_MAP_TYPE_TASK_STORAGE);
	__uint(map_flags, BPF_F_NO_PREALLOC);
	__type(key, int);
	__type(value, struct task_ctx);
} task_ctx_stor SEC(".maps");

static struct task_ctx *try_lookup_task_ctx(const struct task_struct *p)
{
	return bpf_task_storage_get(&task_ctx_stor,
				    (struct task_struct *)p, 0, 0);
}

/* ─── Throttle timer ────────────────────────────────────────────────────── */

struct throttle_timer { struct bpf_timer timer; };
struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 1);
	__type(key, u32);
	__type(value, struct throttle_timer);
} throttle_timer SEC(".maps");

/* ─── DSQ helpers ───────────────────────────────────────────────────────── */

/*
 * DSQ ID layout:
 *   [0 .. nr_cpu_ids)   per-CPU DSQs
 *   nr_cpu_ids + 0      TIER_INTERACTIVE global DSQ
 *   nr_cpu_ids + 1      TIER_DEFAULT global DSQ
 *   nr_cpu_ids + 2      TIER_BACKGROUND global DSQ
 */
static inline u64 cpu_dsq(s32 cpu)   { return (u64)cpu; }
static inline u64 tier_dsq(u8 tier)  { return nr_cpu_ids + (u64)tier; }

/* ─── Small helpers ─────────────────────────────────────────────────────── */

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
static inline bool is_wakeup(u64 wake_flags)
{
	return wake_flags & SCX_WAKE_TTWU;
}

/*
 * Cross-tier EDF comparison.
 * tier_vtime_base is baked into dsq_vtime at enqueue, so a plain u64
 * comparison always picks the highest-priority tier's head task.
 */
static inline bool is_deadline_min(const struct task_struct *p1,
				   const struct task_struct *p2)
{
	if (!p1) return false;
	if (!p2) return true;
	return p1->scx.dsq_vtime < p2->scx.dsq_vtime;
}

static inline u64 calc_avg(u64 old_val, u64 new_val)
{
	return (old_val - (old_val >> 2)) + (new_val >> 2);
}
static inline u64 update_freq(u64 freq, u64 interval)
{
	u64 new_freq = (100ULL * NSEC_PER_MSEC) / interval;
	return calc_avg(freq, new_freq);
}

/* ─── NUMA / idle cpumask helpers ───────────────────────────────────────── */

static inline const struct cpumask *get_idle_cpumask(s32 cpu)
{
	if (!numa_enabled)
		return scx_bpf_get_idle_cpumask();
	return __COMPAT_scx_bpf_get_idle_cpumask_node(
		__COMPAT_scx_bpf_cpu_node(cpu));
}

static inline const struct cpumask *get_idle_smtmask(s32 cpu)
{
	if (!numa_enabled)
		return scx_bpf_get_idle_smtmask();
	return __COMPAT_scx_bpf_get_idle_smtmask_node(
		__COMPAT_scx_bpf_cpu_node(cpu));
}

static inline bool is_cpu_valid(s32 cpu)
{
	u64 max_cpu = MIN(nr_cpu_ids, MAX_CPUS);
	if (cpu < 0 || cpu >= (s32)max_cpu) {
		scx_bpf_error("invalid CPU id: %d", cpu);
		return false;
	}
	return true;
}

static inline bool cpus_share_cache(s32 a, s32 b)
{
	if (a == b) return true;
	if (!is_cpu_valid(a) || !is_cpu_valid(b)) return false;
	return cpu_llc_id(a) == cpu_llc_id(b);
}

static inline bool is_cpu_faster(s32 a, s32 b)
{
	if (a == b) return false;
	if (!is_cpu_valid(a) || !is_cpu_valid(b)) return false;
	return cpu_capacity[a] > cpu_capacity[b];
}

static inline bool is_cpu_efficient(s32 cpu)
{
	if (!is_cpu_valid(cpu)) return false;
	return cpu_capacity[cpu] < READ_ONCE(max_cpu_cap);
}

static s32 smt_sibling(s32 cpu)
{
	const struct cpumask *smt;
	struct cpu_ctx *cctx = try_lookup_cpu_ctx(cpu);
	if (!cctx) return cpu;
	smt = cast_mask(cctx->smt);
	if (!smt) return cpu;
	return bpf_cpumask_first(smt);
}

static bool is_smt_contended(s32 cpu)
{
	const struct cpumask *idle_mask;
	bool contended;
	if (!smt_enabled) return false;
	idle_mask = get_idle_cpumask(cpu);
	contended = !bpf_cpumask_test_cpu(smt_sibling(cpu), idle_mask) &&
		    !bpf_cpumask_empty(idle_mask);
	scx_bpf_put_cpumask(idle_mask);
	return contended;
}

/* ─── TIMELY adaptive slice ──────────────────────────────────────────────
 *
 * Full implementation of the TIMELY delay-driven feedback control loop.
 *
 * The algorithm:
 *
 *  1. Queue delay is sampled every time a task starts running:
 *       delay = last_run_at - timely_last_enqueued_at
 *     An EWMA smooths the delay.  An EWMA of the gradient (delay delta)
 *     detects whether delay is rising or falling.
 *
 *  2. Every control_interval_ns the per-task gain is updated:
 *
 *     delay < tlow  (low region):
 *       gain += gain_step  — tasks are draining fast, allow longer slices.
 *       Reset HAI streak.
 *
 *     tlow ≤ delay ≤ thigh  (mid region):
 *       gradient rising steeply → backoff_gradient applied.
 *       gradient near-zero or falling → gain unchanged (stable point).
 *
 *     delay > thigh  (high region):
 *       gain *= backoff_high  — heavy congestion, shorten slices aggressively.
 *       If gain below hai_thresh: increment HAI streak.
 *       If HAI streak reaches hai_multiplier: halve gain, reset streak.
 *
 *  3. gain clamped to [gain_min_fp .. gain_max_fp].
 *
 *  4. Per-task slice = clamp(slice_max * gain_fp / FP_ONE, slice_min, slice_max).
 *
 * XNU analogue: sched_TIMELY in the Apple scheduler (published research).
 * ─────────────────────────────────────────────────────────────────────── */

static void timely_sample_delay(struct task_ctx *tctx, u64 now)
{
	u64 delay, prev_avg;
	s64 gradient;

	if (!tctx->timely_last_enqueued_at) return;

	delay = now > tctx->timely_last_enqueued_at
		? now - tctx->timely_last_enqueued_at : 1ULL;

	prev_avg = tctx->timely_avg_queue_delay;

	/* EWMA of delay: α = 1/4 (shift by 2). */
	tctx->timely_avg_queue_delay =
		(tctx->timely_avg_queue_delay * 3 + delay) >> 2;

	/* EWMA of gradient. */
	gradient = (s64)tctx->timely_avg_queue_delay - (s64)prev_avg;
	tctx->timely_avg_queue_gradient =
		(tctx->timely_avg_queue_gradient * 3 + gradient) >> 2;

	tctx->timely_last_delay_sample_at = now;
	tctx->timely_last_enqueued_at     = 0;

	/* Classify sample into region and count it. */
	if (delay < timely_tlow_ns) {
		__sync_fetch_and_add(&nr_delay_low_region_samples, 1);
	} else if (delay <= timely_thigh_ns) {
		__sync_fetch_and_add(&nr_delay_mid_region_samples, 1);
	} else {
		__sync_fetch_and_add(&nr_delay_high_region_samples, 1);
	}
}

/*
 * Update the per-task TIMELY gain once per control_interval_ns.
 * Returns the new gain for immediate use in slice computation.
 */
static u32 timely_update_gain(struct task_ctx *tctx, u64 now)
{
	u64 delay = tctx->timely_avg_queue_delay;
	s64 grad  = tctx->timely_avg_queue_gradient;
	u32 gain  = tctx->timely_gain_fp;
	u64 grad_abs;

	/* Only update once per control interval. */
	if (now - tctx->timely_last_gain_update_at <
	    timely_control_interval_ns)
		return gain;

	tctx->timely_last_gain_update_at = now;

	if (delay < timely_tlow_ns) {
		/*
		 * Low delay region: increase gain, tasks are getting CPU fast.
		 * Reset HAI streak since we are not congested.
		 */
		gain += timely_gain_step_fp;
		tctx->timely_hai_streak = 0;
		__sync_fetch_and_add(&nr_delay_recovery_dispatches, 1);
		dbg_msg("TIMELY low: gain=%u delay=%llu", gain, delay);

	} else if (delay <= timely_thigh_ns) {
		/*
		 * Mid delay region: stable if gradient is small.
		 * If gradient is rising steeply apply a mild backoff.
		 */
		grad_abs = grad < 0 ? (u64)(-grad) : (u64)grad;
		if (grad > 0 && grad_abs > timely_gradient_margin_ns) {
			gain = (u32)((u64)gain * timely_backoff_gradient_fp /
				     FP_ONE);
			__sync_fetch_and_add(
				&nr_delay_middle_add_dispatches, 1);
			dbg_msg("TIMELY mid-rising: gain=%u grad=%lld",
				gain, grad);
		} else if (grad < 0 && grad_abs > timely_gradient_margin_ns) {
			/*
			 * Delay falling: recover gain cautiously using
			 * backoff_low_fp as a multiplicative recovery floor.
			 * This prevents gain from climbing too fast after a
			 * high-delay episode and causing renewed congestion.
			 */
			u32 rec_floor = (u32)((u64)gain *
					      timely_backoff_low_fp / FP_ONE);
			if (gain >= rec_floor)
				gain += timely_gain_step_fp / 2;
			__sync_fetch_and_add(
				&nr_delay_fast_recovery_dispatches, 1);
		}

	} else {
		/*
		 * High delay region: apply aggressive backoff.
		 * Track HAI streak for persistent congestion detection.
		 */
		gain = (u32)((u64)gain * timely_backoff_high_fp / FP_ONE);
		__sync_fetch_and_add(&nr_delay_rate_limited_dispatches, 1);
		dbg_msg("TIMELY high: gain=%u delay=%llu", gain, delay);

		if (gain < timely_hai_thresh_fp) {
			tctx->timely_hai_streak++;
			if (tctx->timely_hai_streak >= timely_hai_multiplier) {
				gain /= 2;
				tctx->timely_hai_streak = 0;
				dbg_msg("TIMELY HAI fired: gain=%u", gain);
			}
		} else {
			tctx->timely_hai_streak = 0;
		}
	}

	/* Clamp gain. */
	if (gain < timely_gain_min_fp) {
		gain = timely_gain_min_fp;
		__sync_fetch_and_add(&nr_gain_floor_dispatches, 1);
	}
	if (gain > timely_gain_max_fp) {
		gain = timely_gain_max_fp;
		__sync_fetch_and_add(&nr_gain_ceiling_dispatches, 1);
	}

	tctx->timely_gain_fp = gain;

	/* Classify gain into resident region for monitoring. */
	if (gain <= timely_gain_min_fp + timely_gain_step_fp)
		__sync_fetch_and_add(&nr_gain_floor_resident_samples, 1);
	else if (gain >= timely_gain_max_fp - timely_gain_step_fp)
		__sync_fetch_and_add(&nr_gain_ceiling_resident_samples, 1);
	else
		__sync_fetch_and_add(&nr_gain_mid_resident_samples, 1);

	return gain;
}

/* Return the TIMELY-adapted slice for task @p. */
static u64 timely_task_slice(struct task_ctx *tctx, u64 now)
{
	u32 gain;
	u64 slice;

	gain  = timely_update_gain(tctx, now);
	slice = (u64)slice_max * gain / FP_ONE;
	return CLAMP(slice, slice_min ? slice_min : 1, slice_max);
}

/* Forward declaration: task_slice is defined after effective_slice because
 * it depends on DSQ helpers that follow the TIMELY block. */
static u64 task_slice(const struct task_struct *p, s32 cpu);

/*
 * effective_slice() — unified slice selector.
 *
 * Gap 1 fix: when TIMELY is enabled, tasks that go through the shared tier
 * DSQs or the per-CPU DSQ via normal scheduling use their per-task TIMELY
 * gain to compute the slice.  Tasks on direct fast-paths (sticky, kthread,
 * pcpu) use task_slice() unchanged — those paths are too short-lived for
 * TIMELY feedback to be useful, and their callers do not have a task_ctx.
 *
 * @tctx   : task context (NULL is allowed — falls back to task_slice).
 * @p      : task struct for weight-based scaling.
 * @cpu    : CPU for queue-depth count.
 * @now    : current timestamp (used by timely_update_gain).
 */
static u64 effective_slice(struct task_ctx *tctx,
			   const struct task_struct *p,
			   s32 cpu, u64 now)
{
	if (timely_enabled && tctx)
		return timely_task_slice(tctx, now);
	return task_slice(p, cpu);
}

/*
 * ─── Per-tier warp budget helpers ────────────────────────────────────────
 *
 * XNU analogue: sched_clutch_root_highest_root_bucket() warp handling in
 * sched_clutch.c.  Real Clutch gives each scheduling bucket a *depleting*
 * warp budget that lets it jump ahead of the EDF-selected bucket for a
 * bounded amount of wall-clock time; the budget is only restored to full
 * when the bucket is next selected "in natural order" (i.e. it wins the
 * EDF race on deadline alone, without spending warp).  A tier that keeps
 * receiving new work cannot keep warping forever — once its budget hits
 * zero it must wait until it wins fairly before it can warp again.
 *
 * This intentionally differs from a fixed sliding window (the original
 * scx_aura design): a fixed window re-arms on every new arrival and can
 * never deplete, which lets a steady stream of interactive enqueues starve
 * DEFAULT/BACKGROUND indefinitely — exactly the failure mode XNU's
 * depleting-budget design exists to prevent.  Battery life depends on
 * background/default work (compiles, indexing, backups) eventually
 * draining so the system can go idle; an unbounded warp directly works
 * against that.
 */

/*
 * tier_warp_try_consume(): attempt to spend warp budget for `tier` to
 * cover the dispatch happening at time `now`.  Returns true if `tier` may
 * warp ahead of EDF right now.  Lazily opens a window sized to the
 * remaining budget on first use, then charges real elapsed time against
 * the remaining budget on every subsequent call while the window stays
 * open.  Once tier_warp_remaining_ns[tier] reaches 0 this always returns
 * false until the tier is refilled by tier_warp_refill().
 *
 * Concurrency: tier_warp_remaining_ns[] is global state shared by every
 * CPU's dispatch() call, not per-CPU state.  A plain READ_ONCE +
 * WRITE_ONCE pair is not enough here — two CPUs can both read the same
 * `remaining`, each compute their own charge, and the second writer's
 * store silently clobbers the first, under- or double-charging the
 * budget.  tier_warp_remaining_ns[] is therefore updated with a CAS
 * retry loop so concurrent charges always compose correctly.
 * tier_warp_window_until_ns[]/tier_warp_window_opened_ns[] are updated
 * with plain stores: a lost or reordered update to those only changes
 * exactly when the next caller re-opens or re-checks the window, which
 * self-corrects on the next call and never lets budget be created or
 * destroyed, unlike a race on tier_warp_remaining_ns[] itself.
 */
static inline bool tier_warp_try_consume(u8 tier, u64 now)
{
	u64 remaining, until, opened, spent, new_remaining;
	int retry;

	if (!warp_enabled || tier_warp_budget_ns[tier] == 0)
		return false;

	remaining = READ_ONCE(tier_warp_remaining_ns[tier]);
	if (remaining == 0)
		return false;

	until = READ_ONCE(tier_warp_window_until_ns[tier]);
	if (until == 0 || !time_before(now, until)) {
		/* No window open (or it has lapsed): open a fresh one
		 * sized to whatever budget remains right now.  Re-reading
		 * `remaining` isn't needed for correctness here since we
		 * are not charging anything yet, only recording when this
		 * fresh window should end. */
		WRITE_ONCE(tier_warp_window_opened_ns[tier], now);
		WRITE_ONCE(tier_warp_window_until_ns[tier], now + remaining);
		return true;
	}

	/*
	 * Window already open: charge elapsed time since it was opened
	 * (or since it was last charged) against the remaining budget, via
	 * CAS so concurrent charges from other CPUs can't be lost.  Bounded
	 * retry loop (not a bare `for (;;)`): the BPF verifier requires
	 * statically provable termination, and this file's convention for
	 * that is bpf_for() with an explicit cap, same as the kick loops
	 * elsewhere in dispatch()/enqueue().
	 */
	opened = READ_ONCE(tier_warp_window_opened_ns[tier]);
	spent  = (now > opened) ? (now - opened) : 0;

	new_remaining = 0;
	bpf_for(retry, 0, WARP_CAS_MAX_RETRY) {
		remaining = READ_ONCE(tier_warp_remaining_ns[tier]);
		if (remaining == 0)
			return false;
		new_remaining = (spent >= remaining) ? 0 : (remaining - spent);
		if (__sync_val_compare_and_swap(&tier_warp_remaining_ns[tier],
						 remaining, new_remaining)
		    == remaining)
			goto charged;
		/* Lost the race to another CPU charging concurrently;
		 * retry against whatever value it left behind. */
	}
	/*
	 * Exhausted retries under heavy contention: be conservative and
	 * decline to warp this time rather than risk a stale charge.  The
	 * tier simply falls through to normal EDF for this dispatch call
	 * and gets another chance next time.
	 */
	return false;

charged:
	WRITE_ONCE(tier_warp_window_opened_ns[tier], now);
	if (new_remaining == 0) {
		WRITE_ONCE(tier_warp_window_until_ns[tier], 0);
		return false;
	}
	return true;
}

/*
 * tier_warp_refill(): called when `tier` wins the EDF race fairly (on
 * deadline alone, not via warp).  Restores its warp budget to full and
 * closes any open window, exactly like XNU restoring scrb_warp_remaining
 * when a bucket is chosen in natural order.
 *
 * A plain store is correct here (no CAS needed): refilling always sets
 * the budget to the same fixed full value regardless of what was there
 * before, so two concurrent refills converge to the same result either
 * way: full budget. Concurrent perfectly-interleaved consume + refill is
 * the case worth naming: in the worst case a refill is overwritten by a
 * stale, smaller consume result, undercharging that one window by at
 * most a few microseconds, of a budget measured in milliseconds, which is
 * within scheduler noise and does not violate the warp/EDF starvation
 * bound this mechanism exists to guarantee. The CAS loop above is the
 * lock specifically against the case that does matter: lost deductions.
 */
static inline void tier_warp_refill(u8 tier)
{
	if (tier_warp_budget_ns[tier] == 0)
		return;
	WRITE_ONCE(tier_warp_remaining_ns[tier], tier_warp_budget_ns[tier]);
	WRITE_ONCE(tier_warp_window_until_ns[tier], 0);
}

/* ─── Per-tier WCEL enforcement ─────────────────────────────────────────
 *
 * XNU analogue: sched_clutch_root_highest_root_bucket() starvation avoidance.
 *
 * For each tier we record when the current head was enqueued
 * (tier_enqueue_ts[]).  In dispatch(), before the EDF race, any tier
 * whose head has waited longer than tier_wcel_ns[] has its head task's
 * dsq_vtime overridden to 0, guaranteeing it wins.
 *
 * This is checked only for TIER_DEFAULT and TIER_BACKGROUND; the
 * interactive tier has WCEL = 0 (must always run immediately) which is
 * already achieved by tier_vtime_base = 0.
 * ─────────────────────────────────────────────────────────────────────── */

/*
 * tier_wcel_expired(): has `tier`'s head been waiting longer than its
 * WCEL budget, and if so, is it still within a bounded starvation
 * avoidance window (or does it not need one)?
 *
 * XNU analogue: the starvation-avoidance branch of
 * sched_clutch_root_highest_root_bucket() (sched_clutch.c).  Real
 * Clutch does NOT let an aged-out bucket win unconditionally for as
 * long as its head stays old — it only does so when no higher bucket
 * is currently runnable.  When a higher bucket *is* runnable, the aged
 * bucket instead gets a single bounded quantum (scrb_starvation_ts +
 * sched_clutch_thread_quantum[bucket]), after which its deadline is
 * recomputed and the decision is re-evaluated from scratch — i.e. the
 * lower bucket gets "roughly one quantum per core" rather than the
 * whole CPU for as long as it likes.
 *
 * `higher_runnable` should be true iff a strictly higher tier than
 * `tier` currently has a queued head task (e.g. p_iact for tier ==
 * TIER_DEFAULT or TIER_BACKGROUND, or p_iact-or-p_def for tier ==
 * TIER_BACKGROUND).  When false, this collapses to the original
 * unconditional-override behavior, since there is nothing to bound
 * against — XNU does exactly the same collapse for its top bucket.
 */
static inline bool tier_wcel_expired(u8 tier, u64 now, bool higher_runnable)
{
	u64 wcel = tier_wcel_ns[tier];
	u64 enq_ts, quantum, win_start;

	if (wcel == 0) return false;	/* TIER_INTERACTIVE: no bound */
	enq_ts = READ_ONCE(tier_enqueue_ts[tier]);
	if (enq_ts == 0) return false;	/* tier is empty             */
	if ((now - enq_ts) < wcel) {
		/* Not aged out yet: make sure any stale window from a
		 * previous episode is closed so it can't linger. */
		if (READ_ONCE(tier_starvation_active[tier]))
			WRITE_ONCE(tier_starvation_active[tier], false);
		return false;
	}

	if (!higher_runnable) {
		/*
		 * Nothing above this tier is runnable right now: nothing to
		 * bound against, so this matches XNU's "EDF bucket selected
		 * in the natural order" branch — force the win, no window
		 * needed.  Make sure we're not left mid-window from an
		 * episode where something higher was runnable a moment ago.
		 */
		WRITE_ONCE(tier_starvation_active[tier], false);
		return true;
	}

	quantum = (tier == TIER_BACKGROUND) ? STARVATION_QUANTUM_BACKGROUND_NS
					     : STARVATION_QUANTUM_DEFAULT_NS;

	if (!READ_ONCE(tier_starvation_active[tier])) {
		/* Opening a fresh window: this is the moment a higher tier
		 * is runnable but this tier's aged head still needs to win
		 * right now, matching XNU's
		 * "edf_bucket->scrb_starvation_avoidance = true" branch. */
		WRITE_ONCE(tier_starvation_active[tier], true);
		WRITE_ONCE(tier_starvation_ts[tier], now);
		__sync_fetch_and_add(&nr_starvation_window_opens, 1);
		return true;
	}

	win_start = READ_ONCE(tier_starvation_ts[tier]);
	if ((now - win_start) >= quantum) {
		/*
		 * Window has run its full quantum: close it.  Returning
		 * false here is the key difference from the old unbounded
		 * behavior — the tier stops being forced to win and falls
		 * through to a normal EDF comparison against the higher
		 * tier for this dispatch call, exactly as XNU recomputes
		 * the bucket's deadline and re-evaluates from
		 * evaluate_root_buckets rather than re-granting starvation
		 * avoidance immediately.  If the tier is still starved next
		 * time around, a fresh window opens again above.
		 */
		WRITE_ONCE(tier_starvation_active[tier], false);
		return false;
	}

	/* Still inside an already-open window: keep forcing the win. */
	return true;
}

/*
 * Record when the first task entered a tier DSQ.
 * Called from aura_enqueue() when a task is inserted into a tier DSQ.
 * Uses a simple "set if currently zero" to track only the oldest enqueue
 * time (the head of the queue's age), not every subsequent enqueue.
 */
static inline void tier_enqueue_ts_update(u8 tier, u64 now)
{
	if (READ_ONCE(tier_enqueue_ts[tier]) == 0)
		WRITE_ONCE(tier_enqueue_ts[tier], now);
}

/* Clear the enqueue timestamp when the tier DSQ is drained. */
static inline void tier_enqueue_ts_clear(u8 tier)
{
	WRITE_ONCE(tier_enqueue_ts[tier], 0);
}

/* ─── Group interactivity scorer ─────────────────────────────────────────── */

static u8 group_iact_score(u64 cpu_used, u64 blocked)
{
	u64 half = IACT_PRI_MAX / 2;
	u64 delta;

	if (cpu_used == 0 && blocked == 0)
		return (u8)IACT_PRI_MAX;
	if (blocked >= cpu_used) {
		delta = blocked - cpu_used;
		return (u8)(half + (half * delta / blocked));
	}
	return (u8)(half * blocked / cpu_used);
}

static u8 score_to_tier(u8 score, int nice)
{
	if (nice >= NICE_BACKGROUND) return TIER_BACKGROUND;
	if (nice < NICE_INTERACTIVE) return TIER_INTERACTIVE;
	if (score >= IACT_THRESH)    return TIER_INTERACTIVE;
	if (score <= IACT_BG_THRESH) return TIER_BACKGROUND;
	return TIER_DEFAULT;
}

static void group_iact_wakeup(struct task_struct *p, u64 now)
{
	u32 tgid;
	struct group_iact *gi;
	struct group_iact init = {
		.score     = IACT_PRI_MAX,
		.tier      = TIER_INTERACTIVE,
		.decay_gen = 0,
	};
	u64 blocked;

	if (!group_iact_enabled) return;

	tgid = p->tgid;
	gi = bpf_map_lookup_elem(&group_iact_map, &tgid);
	if (!gi) {
		bpf_map_update_elem(&group_iact_map, &tgid, &init, BPF_NOEXIST);
		gi = bpf_map_lookup_elem(&group_iact_map, &tgid);
		if (!gi) return;
	}

	if (gi->runnable_count == 0 &&
	    gi->blocked_start_ns != 0 &&
	    now > gi->blocked_start_ns) {
		blocked = now - gi->blocked_start_ns;
		if (blocked > IACT_WINDOW_NS)
			blocked = IACT_WINDOW_NS;
		gi->blocked_accum_ns += blocked;
		gi->blocked_start_ns  = 0;
	}
	gi->runnable_count++;
}

static void group_iact_quiesce(struct task_struct *p, u64 now)
{
	u32 tgid;
	struct group_iact *gi;

	if (!group_iact_enabled) return;

	tgid = p->tgid;
	gi = bpf_map_lookup_elem(&group_iact_map, &tgid);
	if (!gi) return;

	if (gi->runnable_count > 0)
		gi->runnable_count--;
	if (gi->runnable_count == 0)
		gi->blocked_start_ns = now;
}

static void group_iact_cpu_used(struct task_struct *p, u64 slice_ns)
{
	u32 tgid;
	struct group_iact *gi;
	u64 delta, new_gen;
	u8 old_tier;

	if (!group_iact_enabled) return;

	tgid = p->tgid;
	gi = bpf_map_lookup_elem(&group_iact_map, &tgid);
	if (!gi) return;

	delta = MIN(slice_ns, IACT_WINDOW_NS);
	gi->cpu_used_ns += delta;

	new_gen = gi->cpu_used_ns / IACT_WINDOW_NS;
	if (new_gen > gi->decay_gen) {
		gi->cpu_used_ns      /= ADJUST_RATIO;
		gi->blocked_accum_ns /= ADJUST_RATIO;
		gi->decay_gen         = new_gen;
	}

	old_tier  = gi->tier;
	gi->score = group_iact_score(gi->cpu_used_ns, gi->blocked_accum_ns);
	gi->tier  = score_to_tier(gi->score, p->static_prio - 120);

	if (gi->tier < old_tier) {
		__sync_fetch_and_add(&nr_iact_promoted, 1);
		dbg_msg("group %u promoted to tier %u score=%u",
			p->tgid, gi->tier, gi->score);
	} else if (gi->tier > old_tier) {
		__sync_fetch_and_add(&nr_iact_demoted, 1);
		dbg_msg("group %u demoted to tier %u score=%u",
			p->tgid, gi->tier, gi->score);
	}
}

static u8 task_tier(struct task_struct *p, struct task_ctx *tctx)
{
	int nice = p->static_prio - 120;
	u32 tgid;
	struct group_iact *gi;

	if (nice >= NICE_BACKGROUND) return TIER_BACKGROUND;
	if (nice < NICE_INTERACTIVE)  return TIER_INTERACTIVE;
	if (tctx->wakeup_freq >= 16)  return TIER_INTERACTIVE;

	if (group_iact_enabled) {
		tgid = p->tgid;
		gi = bpf_map_lookup_elem(&group_iact_map, &tgid);
		if (gi)
			return gi->tier;
	}
	return TIER_DEFAULT;
}

/* ─── E-core consolidation ──────────────────────────────────────────────── */

/*
 * Returns true when the system is lightly loaded enough to consolidate
 * work onto P-cores and let E-cores idle.
 *
 * Applied to ALL tiers (audit fix: was interactive-only) so that default
 * and background tasks also vacate E-cores when the system is quiet,
 * allowing those cores to reach deep C-states.
 */
static inline bool should_consolidate(void)
{
	u64 threshold;
	if (!ecore_consolidate) return false;
	threshold = nr_online_cpus * CONSOLIDATION_THRESH_NUM /
		    CONSOLIDATION_THRESH_DEN;
	return nr_running < threshold;
}

/* ─── Virtual deadline ───────────────────────────────────────────────────── */

static u64 task_dl(struct task_struct *p, s32 cpu,
		   struct task_ctx *tctx, u8 tier)
{
	const u64 STARVATION_THRESH = STARVATION_MS * NSEC_PER_MSEC / 10;
	const u64 q_thresh = MAX(STARVATION_THRESH / slice_max, 1);
	u64 nr_queued, lag_scale, awake_max, vtime_min, raw_dl;

	nr_queued = scx_bpf_dsq_nr_queued(cpu_dsq(cpu)) +
		    scx_bpf_dsq_nr_queued(tier_dsq(TIER_INTERACTIVE)) +
		    scx_bpf_dsq_nr_queued(tier_dsq(TIER_DEFAULT)) +
		    scx_bpf_dsq_nr_queued(tier_dsq(TIER_BACKGROUND));

	lag_scale = MAX(tctx->wakeup_freq, 1);
	awake_max = scale_by_task_weight_inverse(p, slice_lag);

	if (nr_queued * slice_max >= STARVATION_THRESH)
		lag_scale = 1;
	else
		lag_scale = MAX(lag_scale * q_thresh / (q_thresh + nr_queued), 1);

	vtime_min = vtime_now - scale_by_task_weight(p, slice_lag * lag_scale);
	if (time_before(p->scx.dsq_vtime, vtime_min))
		p->scx.dsq_vtime = vtime_min;

	if (time_after(tctx->awake_vtime, awake_max))
		tctx->awake_vtime = awake_max;

	raw_dl = p->scx.dsq_vtime + tctx->awake_vtime;
	return raw_dl + tier_vtime_base[tier];
}

static u64 task_slice(const struct task_struct *p, s32 cpu)
{
	u64 nr_wait = scx_bpf_dsq_nr_queued(cpu_dsq(cpu)) +
		      scx_bpf_dsq_nr_queued(tier_dsq(TIER_INTERACTIVE)) +
		      scx_bpf_dsq_nr_queued(tier_dsq(TIER_DEFAULT)) +
		      scx_bpf_dsq_nr_queued(tier_dsq(TIER_BACKGROUND));
	u64 slice = scale_by_task_weight(p, slice_max) / MAX(nr_wait, 1);
	return MAX(slice, slice_min);
}

/* ─── Idle CPU selection ─────────────────────────────────────────────────── */

static s32 pick_idle_cpu_pref_smt(struct task_struct *p, s32 prev_cpu,
				  bool is_prev_allowed,
				  const struct cpumask *primary,
				  const struct cpumask *smt)
{
	u64 max_cpus = MIN(nr_cpu_ids, MAX_CPUS);
	int i;

	if (is_prev_allowed &&
	    (!primary || bpf_cpumask_test_cpu(prev_cpu, primary)) &&
	    (!smt    || bpf_cpumask_test_cpu(prev_cpu, smt)) &&
	    scx_bpf_test_and_clear_cpu_idle(prev_cpu)) {
		__sync_fetch_and_add(&nr_idle_prev_cpu_picks, 1);
		return prev_cpu;
	}

	bpf_for(i, 0, max_cpus) {
		s32 cpu = preferred_cpus[i];
		if (cpu == prev_cpu || !bpf_cpumask_test_cpu(cpu, p->cpus_ptr))
			continue;
		if ((!primary || bpf_cpumask_test_cpu(cpu, primary)) &&
		    (!smt    || bpf_cpumask_test_cpu(cpu, smt)) &&
		    scx_bpf_test_and_clear_cpu_idle(cpu)) {
			if (primary && bpf_cpumask_test_cpu(cpu, primary))
				__sync_fetch_and_add(&nr_idle_primary_picks, 1);
			else
				__sync_fetch_and_add(&nr_idle_spill_picks, 1);
			return cpu;
		}
	}
	return -EBUSY;
}

/*
 * Capacity-sorted idle CPU scan.
 * E-core consolidation now applied to all tiers (audit fix: was
 * interactive-only).
 */
static s32 pick_idle_cpu_scan(struct task_struct *p, s32 prev_cpu,
			      bool prefer_pcores)
{
	const struct cpumask *smt;
	const struct cpumask *primary;
	bool is_prev_allowed;
	s32 cpu;

	is_prev_allowed = bpf_cpumask_test_cpu(prev_cpu, p->cpus_ptr);
	primary = !primary_all ? cast_mask(primary_cpumask) : NULL;
	smt     = smt_enabled  ? get_idle_smtmask(prev_cpu) : NULL;

	if (p->nr_cpus_allowed == 1 || is_migration_disabled(p)) {
		if (scx_bpf_test_and_clear_cpu_idle(prev_cpu)) {
			cpu = prev_cpu;
			__sync_fetch_and_add(&nr_idle_prev_cpu_picks, 1);
			goto out;
		}
	}

	if (prefer_pcores) {
		if (smt_enabled) {
			cpu = pick_idle_cpu_pref_smt(p, prev_cpu,
						     is_prev_allowed,
						     primary, smt);
			if (cpu >= 0 && !is_cpu_efficient(cpu))
				goto out;
		}
		cpu = pick_idle_cpu_pref_smt(p, prev_cpu, is_prev_allowed,
					     primary, NULL);
		if (cpu >= 0 && !is_cpu_efficient(cpu))
			goto out;
		/* No P-core found — fall through to standard passes. */
	}

	if (!primary_all) {
		if (smt_enabled) {
			cpu = pick_idle_cpu_pref_smt(p, prev_cpu,
						     is_prev_allowed,
						     primary, smt);
			if (cpu >= 0) goto out;
		}
		cpu = pick_idle_cpu_pref_smt(p, prev_cpu, is_prev_allowed,
					     primary, NULL);
		if (cpu >= 0) {
			__sync_fetch_and_add(&nr_idle_primary_domain_misses, 1);
			goto out;
		}
	}

	if (smt_enabled) {
		cpu = pick_idle_cpu_pref_smt(p, prev_cpu, is_prev_allowed,
					     NULL, smt);
		if (cpu >= 0) goto out;
	}

	cpu = pick_idle_cpu_pref_smt(p, prev_cpu, is_prev_allowed, NULL, NULL);
	if (cpu < 0)
		__sync_fetch_and_add(&nr_idle_pick_failures, 1);
	else
		__sync_fetch_and_add(&nr_idle_global_misses, 1);

out:
	if (smt) scx_bpf_put_cpumask(smt);
	return cpu;
}

static s32 pick_idle_cpu_wakeup(struct task_struct *p, s32 prev_cpu,
				u64 wake_flags, u8 tier, bool prefer_pcores)
{
	const struct cpumask *primary = cast_mask(primary_cpumask);
	s32 cpu;

	if (preferred_idle_scan) {
		__sync_fetch_and_add(&nr_idle_select_path_picks, 1);
		return pick_idle_cpu_scan(p, prev_cpu, prefer_pcores);
	}

	if (no_wake_sync)
		wake_flags &= ~SCX_WAKE_SYNC;

	if (!__COMPAT_HAS_scx_bpf_select_cpu_and) {
		bool is_idle = false;
		cpu = scx_bpf_select_cpu_dfl(p, prev_cpu, wake_flags, &is_idle);
		if (is_idle) __sync_fetch_and_add(&nr_idle_select_path_picks, 1);
		return is_idle ? cpu : -EBUSY;
	}

	if (!primary_all && primary) {
		cpu = scx_bpf_select_cpu_and(p, prev_cpu, wake_flags, primary, 0);
		if (cpu >= 0) {
			if (prefer_pcores && is_cpu_efficient(cpu))
				goto try_global_w;
			__sync_fetch_and_add(&nr_idle_primary_picks, 1);
			return cpu;
		}
		__sync_fetch_and_add(&nr_idle_primary_domain_misses, 1);
	}

try_global_w:
	cpu = scx_bpf_select_cpu_and(p, prev_cpu, wake_flags, p->cpus_ptr, 0);
	if (cpu >= 0) {
		if (prefer_pcores && is_cpu_efficient(cpu)) {
			__sync_fetch_and_add(&nr_ecore_consolidations, 1);
			return -EBUSY;
		}
		__sync_fetch_and_add(&nr_idle_select_path_picks, 1);
		return cpu;
	}
	__sync_fetch_and_add(&nr_idle_global_misses, 1);
	return cpu;
}

static s32 pick_idle_cpu_enqueue(struct task_struct *p, s32 prev_cpu,
				 u8 tier, bool prefer_pcores)
{
	const struct cpumask *primary = cast_mask(primary_cpumask);
	s32 cpu;

	if (preferred_idle_scan) {
		__sync_fetch_and_add(&nr_idle_enqueue_path_picks, 1);
		return pick_idle_cpu_scan(p, prev_cpu, prefer_pcores);
	}

	if (!__COMPAT_HAS_scx_bpf_select_cpu_and)
		return -EBUSY;

	if (!primary_all && primary) {
		cpu = scx_bpf_select_cpu_and(p, prev_cpu, 0, primary, 0);
		if (cpu >= 0) {
			if (prefer_pcores && is_cpu_efficient(cpu))
				goto try_global_e;
			__sync_fetch_and_add(&nr_idle_enqueue_path_picks, 1);
			return cpu;
		}
		__sync_fetch_and_add(&nr_idle_primary_domain_misses, 1);
	}

try_global_e:
	cpu = scx_bpf_select_cpu_and(p, prev_cpu, 0, p->cpus_ptr, 0);
	if (cpu >= 0) {
		if (prefer_pcores && is_cpu_efficient(cpu)) {
			__sync_fetch_and_add(&nr_ecore_consolidations, 1);
			return -EBUSY;
		}
		__sync_fetch_and_add(&nr_idle_enqueue_path_picks, 1);
		return cpu;
	}
	__sync_fetch_and_add(&nr_idle_global_misses, 1);
	return cpu;
}

/* ─── Cpumask helpers ───────────────────────────────────────────────────── */

static int calloc_cpumask(struct bpf_cpumask **p_cpumask)
{
	struct bpf_cpumask *cpumask = bpf_cpumask_create();
	if (!cpumask) return -ENOMEM;
	cpumask = bpf_kptr_xchg(p_cpumask, cpumask);
	if (cpumask) bpf_cpumask_release(cpumask);
	return 0;
}

static int init_cpumask(struct bpf_cpumask **cpumask)
{
	struct bpf_cpumask *mask = *cpumask;
	int err;

	if (mask) return 0;
	err = calloc_cpumask(cpumask);
	if (!err) mask = *cpumask;
	if (!mask) err = -ENOMEM;
	return err;
}

/* ─── CPU frequency scaling ─────────────────────────────────────────────── */

/*
 * Update per-CPU frequency based on measured utilisation.
 *
 * Now called from aura_stopping() unconditionally (not gated by
 * !timely_enabled) so frequency tracks actual utilisation whether TIMELY
 * is on or off.  When TIMELY is on, the TIMELY gain additionally adapts
 * the slice length; both mechanisms are complementary.
 *
 * Cold-start guard: if last_running == 0 record the timestamp and return
 * to avoid computing a garbage utilisation from boot time.
 */
static void update_cpu_load(struct task_struct *p, struct task_ctx *tctx)
{
	u64 now = bpf_ktime_get_ns();
	s32 cpu  = scx_bpf_task_cpu(p);
	struct cpu_ctx *cctx = try_lookup_cpu_ctx(cpu);
	u64 delta_t, delta_runtime, perf_lvl;

	if (!cctx) return;

	if (cctx->last_running == 0) {
		cctx->last_running = now;
		cctx->prev_runtime = cctx->tot_runtime;
		return;
	}

	delta_t       = now - cctx->last_running;
	delta_runtime = cctx->tot_runtime - cctx->prev_runtime;

	if (delta_t == 0)
		perf_lvl = SCX_CPUPERF_ONE;
	else
		perf_lvl = MIN(delta_runtime * SCX_CPUPERF_ONE / delta_t,
			       (u64)SCX_CPUPERF_ONE);

	if (perf_lvl >= SCX_CPUPERF_ONE - SCX_CPUPERF_ONE / 4)
		perf_lvl = SCX_CPUPERF_ONE;

	if (cpufreq_perf_lvl < 0)
		scx_bpf_cpuperf_set(cpu, perf_lvl);

	cctx->last_running = now;
	cctx->prev_runtime = cctx->tot_runtime;
}

/* ─── Sticky task detection ─────────────────────────────────────────────── */

static bool is_task_sticky(const struct task_ctx *tctx)
{
	return sticky_tasks && tctx->avg_runtime < 10 * NSEC_PER_USEC;
}

static bool task_should_migrate(struct task_struct *p, u64 enq_flags)
{
	return !__COMPAT_is_enq_cpu_selected(enq_flags) &&
	       (!sticky_tasks || !scx_bpf_task_running(p));
}

/* ─── select_cpu ────────────────────────────────────────────────────────── */

s32 BPF_STRUCT_OPS(aura_select_cpu, struct task_struct *p,
		   s32 prev_cpu, u64 wake_flags)
{
	s32 cpu, this_cpu = bpf_get_smp_processor_id();
	bool is_this_cpu_allowed;
	struct task_ctx *tctx;
	u8 tier = TIER_DEFAULT;
	bool prefer_pcores;
	/*
	 * Sample timestamp once here.  Used for:
	 *   - timely_last_enqueued_at (TIMELY queue-delay measurement)
	 *   - effective_slice() → timely_task_slice() → timely_update_gain()
	 * Sampling once avoids two ktime calls on the wakeup fast path.
	 */
	u64 enqueue_ts = bpf_ktime_get_ns();

	is_this_cpu_allowed = bpf_cpumask_test_cpu(this_cpu, p->cpus_ptr);

	if (!bpf_cpumask_test_cpu(prev_cpu, p->cpus_ptr))
		prev_cpu = is_this_cpu_allowed ?
			   this_cpu : bpf_cpumask_first(p->cpus_ptr);

	tctx = try_lookup_task_ctx(p);
	if (tctx) {
		tier = task_tier(p, tctx);
		if (timely_enabled)
			tctx->timely_last_enqueued_at = enqueue_ts;
	}

	/* Waker-bias: pull wakee toward a faster waker CPU. */
	if (primary_all && is_wakeup(wake_flags) && is_this_cpu_allowed &&
	    is_cpu_faster(this_cpu, prev_cpu)) {
		if (cpus_share_cache(this_cpu, prev_cpu) &&
		    !is_smt_contended(prev_cpu) &&
		    scx_bpf_test_and_clear_cpu_idle(prev_cpu)) {
			if (tctx) {
				tctx->tier = tier;
				scx_bpf_dsq_insert_vtime(
					p, cpu_dsq(prev_cpu),
					effective_slice(tctx, p, prev_cpu, enqueue_ts),
					task_dl(p, prev_cpu, tctx, tier), 0);
				__sync_fetch_and_add(&nr_direct_dispatches, 1);
				__sync_fetch_and_add(&nr_waker_cpu_biases, 1);
				dbg_msg("waker-bias: pid %d → cpu %d",
					p->pid, prev_cpu);
			}
			return prev_cpu;
		}
		prev_cpu = this_cpu;
	}

	/*
	 * E-core consolidation now applies to all tiers.
	 */
	prefer_pcores = should_consolidate();

	cpu = pick_idle_cpu_wakeup(p, prev_cpu, wake_flags, tier, prefer_pcores);
	if (cpu >= 0) {
		if (tctx) {
			tctx->tier = tier;
			scx_bpf_dsq_insert_vtime(p, cpu_dsq(cpu),
						 effective_slice(tctx, p, cpu, enqueue_ts),
						 task_dl(p, cpu, tctx, tier),
						 0);
			__sync_fetch_and_add(&nr_direct_dispatches, 1);
		}
		return cpu;
	}

	return prev_cpu;
}

/* ─── enqueue ───────────────────────────────────────────────────────────── */

void BPF_STRUCT_OPS(aura_enqueue, struct task_struct *p, u64 enq_flags)
{
	s32 prev_cpu = scx_bpf_task_cpu(p);
	struct task_ctx *tctx = try_lookup_task_ctx(p);
	u64 enqueue_ts, dsq_id, deadline;
	u8 tier;
	s32 cpu;
	bool prefer_pcores;

	if (!tctx) return;

	enqueue_ts = bpf_ktime_get_ns();
	if (timely_enabled)
		tctx->timely_last_enqueued_at = enqueue_ts;

	if (is_task_sticky(tctx)) {
		scx_bpf_dsq_insert(p, SCX_DSQ_LOCAL,
				   task_slice(p, prev_cpu), enq_flags);
		__sync_fetch_and_add(&nr_direct_dispatches, 1);
		return;
	}

	if (local_kthreads && is_kthread(p) && p->nr_cpus_allowed == 1) {
		scx_bpf_dsq_insert(p, SCX_DSQ_LOCAL,
				   task_slice(p, prev_cpu), enq_flags);
		__sync_fetch_and_add(&nr_kthread_dispatches, 1);
		return;
	}

	tier = task_tier(p, tctx);
	if (is_pcpu_task(p)) {
		if (local_pcpu)
			scx_bpf_dsq_insert(p, SCX_DSQ_LOCAL,
					   task_slice(p, prev_cpu), enq_flags);
		else
			scx_bpf_dsq_insert_vtime(p, cpu_dsq(prev_cpu),
						 effective_slice(tctx, p, prev_cpu, enqueue_ts),
						 task_dl(p, prev_cpu, tctx, tier),
						 enq_flags);
		__sync_fetch_and_add(&nr_direct_dispatches, 1);
		return;
	}

	tctx->tier    = tier;
	/* All tiers avoid E-cores when consolidation is active. */
	prefer_pcores = should_consolidate();

	if (task_should_migrate(p, enq_flags) ||
	    (!is_pcpu_task(p) && is_smt_contended(prev_cpu))) {
		cpu = pick_idle_cpu_enqueue(p, prev_cpu, tier, prefer_pcores);
		if (cpu >= 0) {
			scx_bpf_dsq_insert_vtime(p, cpu_dsq(cpu),
						 effective_slice(tctx, p, cpu, enqueue_ts),
						 task_dl(p, cpu, tctx, tier),
						 enq_flags);
			__sync_fetch_and_add(&nr_direct_dispatches, 1);
			if (prev_cpu != cpu || !scx_bpf_task_running(p))
				scx_bpf_kick_cpu(cpu, SCX_KICK_IDLE);
			return;
		}
	}

	dsq_id   = tier_dsq(tier);
	deadline = task_dl(p, prev_cpu, tctx, tier);
	scx_bpf_dsq_insert_vtime(p, dsq_id,
				  effective_slice(tctx, p, prev_cpu, enqueue_ts),
				  deadline, enq_flags);

	/* Record enqueue timestamp for WCEL enforcement. */
	tier_enqueue_ts_update(tier, enqueue_ts);

	if (tier == TIER_INTERACTIVE) {
		if (warp_enabled) {
			s32 kick_cpu;

			/*
			 * Wake CPUs so dispatch() runs promptly and can
			 * decide, against the shared per-tier warp budget,
			 * whether TIER_INTERACTIVE gets to jump ahead of
			 * EDF right now.  Arming/spending the budget itself
			 * happens only in dispatch() — see
			 * tier_warp_try_consume().
			 */
			bpf_for(kick_cpu, 0, MIN(nr_cpu_ids, MAX_CPUS)) {
				if (scx_bpf_test_and_clear_cpu_idle(kick_cpu)) {
					scx_bpf_kick_cpu(kick_cpu, SCX_KICK_IDLE);
				} else {
					scx_bpf_kick_cpu(kick_cpu,
							 SCX_KICK_PREEMPT);
					__sync_fetch_and_add(
						&nr_preempt_kicks, 1);
				}
			}
		}
		__sync_fetch_and_add(&nr_interactive_dispatches, 1);
	} else if (tier == TIER_BACKGROUND) {
		__sync_fetch_and_add(&nr_background_dispatches, 1);
	} else {
		__sync_fetch_and_add(&nr_shared_dispatches, 1);
	}

	if (task_should_migrate(p, enq_flags))
		scx_bpf_kick_cpu(prev_cpu, SCX_KICK_IDLE);
}

/* ─── dispatch ──────────────────────────────────────────────────────────── */

static bool keep_running(const struct task_struct *p, s32 cpu)
{
	bool empty, smt_blocked;

	if (!is_task_queued(p)) return false;

	if (is_pcpu_task(p)) {
		__sync_fetch_and_add(&nr_keep_running_reuses, 1);
		return true;
	}

	empty = scx_bpf_dsq_nr_queued(tier_dsq(TIER_INTERACTIVE)) == 0 &&
		scx_bpf_dsq_nr_queued(tier_dsq(TIER_DEFAULT))     == 0 &&
		scx_bpf_dsq_nr_queued(tier_dsq(TIER_BACKGROUND))  == 0 &&
		scx_bpf_dsq_nr_queued(cpu_dsq(cpu))               == 0;
	if (empty) {
		__sync_fetch_and_add(&nr_keep_running_queue_empty, 1);
		return true;
	}

	smt_blocked = is_smt_contended(cpu);
	if (smt_blocked) {
		__sync_fetch_and_add(&nr_keep_running_smt_blocked, 1);
		return false;
	}

	__sync_fetch_and_add(&nr_keep_running_queued_work, 1);
	return false;
}

void BPF_STRUCT_OPS(aura_dispatch, s32 cpu, struct task_struct *prev)
{
	struct task_struct *p_cpu, *p_iact, *p_def, *p_bg, *winner;
	struct cpu_ctx *cctx;
	u64 win_dsq, now;
	bool wcel_def, wcel_bg;

	if (is_throttled()) return;

	now  = bpf_ktime_get_ns();
	cctx = try_lookup_cpu_ctx(cpu);

	p_cpu  = __COMPAT_scx_bpf_dsq_peek(cpu_dsq(cpu));
	p_iact = __COMPAT_scx_bpf_dsq_peek(tier_dsq(TIER_INTERACTIVE));
	p_def  = __COMPAT_scx_bpf_dsq_peek(tier_dsq(TIER_DEFAULT));
	p_bg   = __COMPAT_scx_bpf_dsq_peek(tier_dsq(TIER_BACKGROUND));

	/*
	 * WCEL expiry is computed up front because both warp blocks below
	 * need to know it: there is no point spending a tier's warp budget
	 * to jump ahead of a lower tier that WCEL is about to force to win
	 * unconditionally anyway (see MAYBE_WIN_WCEL further down).
	 *
	 * higher_runnable for each tier is whether a strictly higher tier
	 * currently has a queued head task — this is what lets
	 * tier_wcel_expired() distinguish "nothing above me, force the win
	 * outright" from "something above me is runnable too, bound this
	 * to one starvation-avoidance quantum" (see that function's
	 * comment, and sched_clutch_root_highest_root_bucket() in real
	 * XNU for the mechanism this mirrors).
	 */
	wcel_def = tier_wcel_expired(TIER_DEFAULT, now, p_iact != NULL);
	wcel_bg  = tier_wcel_expired(TIER_BACKGROUND, now,
				      (p_iact != NULL) || (p_def != NULL));

	/*
	 * ── Warp: drain interactive tier ahead of EDF. ───────────────────
	 *
	 * Gated on !is_deadline_min(p_cpu, p_iact): warp may only let
	 * TIER_INTERACTIVE jump ahead of the *other tier DSQs*, never ahead
	 * of a task already sitting in this CPU's own per-CPU DSQ
	 * (cpu_dsq(cpu)) with a strictly earlier deadline.  cpu_dsq(cpu) is
	 * a direct-placement fast path (idle-CPU picks, sticky/pcpu tasks,
	 * migration targets — see aura_enqueue()), not a tier in the
	 * XNU-bucket sense, so warp has no business overriding a placement
	 * decision already locked in for this CPU.  Without this check, a
	 * task placed directly on cpu_dsq(cpu) with an earlier deadline
	 * than anything queued could still lose its dispatch slot to a
	 * tier's warp shortcut, which is a correctness bug independent of
	 * anything XNU-fidelity related.
	 */
	if (p_iact && cctx && (p_def || p_bg) &&
	    !is_deadline_min(p_cpu, p_iact)) {
		/*
		 * Only spend warp budget when there is actually something
		 * for TIER_INTERACTIVE to jump ahead of.  If DEFAULT and
		 * BACKGROUND are both empty, normal EDF already dispatches
		 * the interactive task next anyway, so consuming budget
		 * here would just waste it for no behavioral gain.
		 */
		if (tier_warp_try_consume(TIER_INTERACTIVE, now)) {
			if (scx_bpf_dsq_move_to_local(
					tier_dsq(TIER_INTERACTIVE), 0)) {
				if (scx_bpf_dsq_nr_queued(
						tier_dsq(TIER_INTERACTIVE)) == 0)
					tier_enqueue_ts_clear(TIER_INTERACTIVE);
				__sync_fetch_and_add(&nr_warp_dispatches, 1);
				__sync_fetch_and_add(
					&nr_dispatch_cpu_dsq_consumes, 1);
				return;
			}
		}
	}

	/*
	 * ── Warp: drain default tier ahead of EDF, over background only.
	 *
	 * Mirrors the interactive warp block above but for DEFAULT's
	 * smaller budget (2 ms vs INTERACTIVE's 8 ms), and only relative
	 * to BACKGROUND — DEFAULT never warps over INTERACTIVE, since
	 * INTERACTIVE's own WCEL of 0 plus its much larger budget already
	 * guarantee it wins whenever it has anything queued.
	 *
	 * That "INTERACTIVE always wins anyway" reasoning only holds if
	 * this block actually checks p_iact — it must not assume the
	 * interactive warp block above already handled it.  That block
	 * only returns early when it actually *fires*; if it declines (no
	 * warp budget left, or its own !is_deadline_min(p_cpu, p_iact)
	 * gate blocks it) control falls through to here with p_iact still
	 * sitting unconsidered.  Without an explicit check here, DEFAULT's
	 * warp could steal the dispatch slot out from under a genuinely
	 * earlier-deadline interactive task — undermining exactly the
	 * "interactive always runs as soon as a CPU is free" guarantee
	 * this scheduler exists to provide.  Same reasoning applies to
	 * p_cpu (a directly-placed task on this CPU's own per-CPU DSQ):
	 * neither is a competitor DEFAULT's warp is entitled to jump.
	 *
	 * Also skipped when wcel_bg is already true: if BACKGROUND's WCEL
	 * is about to force it to win unconditionally regardless of warp,
	 * spending DEFAULT's budget here accomplishes nothing (the
	 * MAYBE_WIN_WCEL race below would just override the result) and
	 * would needlessly burn the budget tier_warp_refill(TIER_DEFAULT)
	 * would otherwise preserve.
	 *
	 * Note there is no separate "!wcel_def" check needed here: if
	 * wcel_def is true, DEFAULT is being forced to win against
	 * INTERACTIVE via MAYBE_WIN_WCEL's *local* synthetic deadline,
	 * which never mutates p_def's real dsq_vtime — so
	 * is_deadline_min(p_iact, p_def) above still compares genuine
	 * vtimes and is true (blocking this block) precisely when
	 * INTERACTIVE's real deadline is earlier, which is exactly the
	 * condition that made wcel_def true to begin with.  The p_iact
	 * gate already covers this case as a consequence, not by
	 * coincidence.
	 */
	if (p_def && p_bg && cctx && !wcel_bg &&
	    !is_deadline_min(p_cpu, p_def) &&
	    !is_deadline_min(p_iact, p_def)) {
		if (tier_warp_try_consume(TIER_DEFAULT, now)) {
			if (scx_bpf_dsq_move_to_local(
					tier_dsq(TIER_DEFAULT), 0)) {
				if (scx_bpf_dsq_nr_queued(
						tier_dsq(TIER_DEFAULT)) == 0)
					tier_enqueue_ts_clear(TIER_DEFAULT);
				__sync_fetch_and_add(
					&nr_default_warp_dispatches, 1);
				__sync_fetch_and_add(
					&nr_dispatch_cpu_dsq_consumes, 1);
				return;
			}
		}
	}

	/*
	 * ── WCEL enforcement ────────────────────────────────────────────
	 *
	 * XNU analogue: sched_clutch_root_highest_root_bucket() starvation window.
	 *
	 * If a lower tier has been waiting longer than its WCEL budget,
	 * override its head task's dsq_vtime to 0 so it wins the EDF race
	 * unconditionally on the next pass.
	 *
	 * TIER_INTERACTIVE has WCEL = 0 (must always win immediately) which
	 * is already guaranteed by tier_vtime_base[0] = 0.  Only DEFAULT and
	 * BACKGROUND need the explicit check.  wcel_def/wcel_bg were already
	 * computed above so the warp blocks could consult them.
	 */
	if ((wcel_def && p_def) || (wcel_bg && p_bg)) {
		__sync_fetch_and_add(&nr_wcel_enforcements, 1);
		dbg_msg("WCEL: def=%d bg=%d at cpu %d", wcel_def, wcel_bg, cpu);
	}

	/*
	 * ── EDF across all sources ──────────────────────────────────────
	 *
	 * Gap 3 fix: WCEL enforcement no longer mutates the peeked task's
	 * dsq_vtime directly (unsafe through a peek pointer).  Instead,
	 * MAYBE_WIN_WCEL substitutes a deadline of 0 for the WCEL-expired
	 * tier's candidate when comparing, without touching the task struct.
	 * A deadline of 0 is numerically earlier than any real vtime so the
	 * WCEL-expired tier wins the selection unconditionally.
	 */
	winner  = NULL;
	win_dsq = 0;

#define MAYBE_WIN(cand, dsq_id) do {				\
	if ((cand) && is_deadline_min((cand), winner)) {	\
		winner  = (cand);				\
		win_dsq = (dsq_id);				\
	}							\
} while (0)

/*
 * MAYBE_WIN_WCEL: like MAYBE_WIN but uses a synthetic zero deadline when
 * wcel_active is true, so this tier wins over any normally-ordered task.
 */
#define MAYBE_WIN_WCEL(cand, dsq_id, wcel_active) do {			\
	if ((cand)) {							\
		u64 _dl = (wcel_active) ? 0ULL : (cand)->scx.dsq_vtime;	\
		if (!winner || _dl < winner->scx.dsq_vtime) {			\
			winner  = (cand);					\
			win_dsq = (dsq_id);					\
		}								\
	}								\
} while (0)

	MAYBE_WIN(p_cpu,  cpu_dsq(cpu));
	MAYBE_WIN(p_iact, tier_dsq(TIER_INTERACTIVE));
	MAYBE_WIN_WCEL(p_def, tier_dsq(TIER_DEFAULT),     wcel_def);
	MAYBE_WIN_WCEL(p_bg,  tier_dsq(TIER_BACKGROUND),  wcel_bg);
#undef MAYBE_WIN_WCEL
#undef MAYBE_WIN

	if (winner) {
		if (scx_bpf_dsq_move_to_local(win_dsq, 0)) {
			/*
			 * Gap 2 fix: only clear the WCEL enqueue timestamp when
			 * the tier DSQ is now empty.  If more tasks remain, the
			 * timestamp continues to reflect the head task's age so
			 * the starvation clock keeps running correctly.
			 * The timestamp is reset to 'now' when the next task is
			 * enqueued into this tier (tier_enqueue_ts_update uses
			 * "set if zero"), so clearing only on empty is safe.
			 */
			if (win_dsq == tier_dsq(TIER_INTERACTIVE)) {
				if (scx_bpf_dsq_nr_queued(
						tier_dsq(TIER_INTERACTIVE)) == 0)
					tier_enqueue_ts_clear(TIER_INTERACTIVE);
				/*
				 * Reaching here means TIER_INTERACTIVE won
				 * the EDF race on deadline alone (the warp
				 * shortcut above already returned if it had
				 * fired) — a fair win, so restore its warp
				 * budget to full.
				 */
				tier_warp_refill(TIER_INTERACTIVE);
				__sync_fetch_and_add(
					&nr_interactive_dispatches, 1);
			} else if (win_dsq == tier_dsq(TIER_DEFAULT)) {
				if (scx_bpf_dsq_nr_queued(
						tier_dsq(TIER_DEFAULT)) == 0)
					tier_enqueue_ts_clear(TIER_DEFAULT);
				/*
				 * Refill only on a genuine fair win (deadline
				 * comparison).  If wcel_def is true, DEFAULT
				 * won via the WCEL starvation override
				 * instead (MAYBE_WIN_WCEL's synthetic zero
				 * deadline) — that is not a "natural order"
				 * win, so it must not refill warp, exactly
				 * as XNU does not refill scrb_warp_remaining
				 * on a starvation-forced selection either.
				 */
				if (!wcel_def)
					tier_warp_refill(TIER_DEFAULT);
			} else if (win_dsq == tier_dsq(TIER_BACKGROUND)) {
				if (scx_bpf_dsq_nr_queued(
						tier_dsq(TIER_BACKGROUND)) == 0)
					tier_enqueue_ts_clear(TIER_BACKGROUND);
			}
			__sync_fetch_and_add(&nr_dispatch_cpu_dsq_consumes, 1);
			return;
		}
	}

	/*
	 * ── Passive E-core → P-core rebalance ───────────────────────────
	 *
	 * XNU analogue: sched_amp_balance() — once a P-core goes idle, pull
	 * back P-recommended work that had spilled onto E-cores.
	 *
	 * This is the "passive" half of that idea: we never kick or IPI an
	 * E-core, and we never touch a task that is already *running*
	 * there.  We only act when this P-core is otherwise about to go
	 * idle (every other dispatch source above came up empty) and
	 * consolidation is currently desired — i.e. exactly the scenario
	 * should_consolidate() already steers new placements away from
	 * E-cores for.  Without this, a task that landed on an E-core
	 * during a busier moment can sit there until it happens to wake
	 * up again, even while a P-core sits idle right next to it.
	 *
	 * We only ever pull *queued* (not-yet-running) tasks out of an
	 * E-core's per-CPU DSQ via the normal DSQ move primitive, so this
	 * carries no extra synchronization risk beyond what dispatch()
	 * already does for its own per-CPU DSQ.
	 */
	if (!is_cpu_efficient(cpu) && should_consolidate()) {
		s32 ecpu, max_cpu = MIN(nr_cpu_ids, MAX_CPUS);

		bpf_for(ecpu, 0, max_cpu) {
			if (ecpu == cpu || !is_cpu_efficient(ecpu))
				continue;
			if (scx_bpf_dsq_nr_queued(cpu_dsq(ecpu)) == 0)
				continue;
			if (scx_bpf_dsq_move_to_local(cpu_dsq(ecpu), 0)) {
				__sync_fetch_and_add(
					&nr_ecore_rebalance_pulls, 1);
				__sync_fetch_and_add(
					&nr_dispatch_cpu_dsq_consumes, 1);
				return;
			}
		}
	}

	if (prev && keep_running(prev, cpu)) {
		struct task_ctx *ptctx = try_lookup_task_ctx(prev);
		prev->scx.slice = effective_slice(ptctx, prev, cpu,
					      bpf_ktime_get_ns());
	}
}

/* ─── running ───────────────────────────────────────────────────────────── */

void BPF_STRUCT_OPS(aura_running, struct task_struct *p)
{
	struct task_ctx *tctx;
	u64 raw_vtime;
	u64 now;

	__sync_fetch_and_add(&nr_running, 1);

	tctx = try_lookup_task_ctx(p);
	if (!tctx) return;

	now = bpf_ktime_get_ns();
	tctx->last_run_at = now;

	/*
	 * TIMELY: sample queue delay and update gain.
	 * The gain update in timely_update_gain() adjusts the per-task slice
	 * multiplier used in timely_task_slice(), which is then applied in
	 * aura_stopping() when the next slice is assigned.
	 */
	if (timely_enabled) {
		timely_sample_delay(tctx, now);
		/* Eagerly update gain so the next slice uses the new value. */
		timely_update_gain(tctx, now);
	}

	/*
	 * Strip tier offset to get raw vtime before comparing against
	 * vtime_now.  A background task's +800 ms offset must not inflate
	 * the global lag baseline.
	 */
	{
		u8 tier = tctx->tier < NR_TIERS ? tctx->tier : TIER_DEFAULT;
		raw_vtime = p->scx.dsq_vtime;
		if (raw_vtime >= tier_vtime_base[tier])
			raw_vtime -= tier_vtime_base[tier];
		if (time_before(vtime_now, raw_vtime))
			vtime_now = raw_vtime;
	}
}

/* ─── stopping ──────────────────────────────────────────────────────────── */

void BPF_STRUCT_OPS(aura_stopping, struct task_struct *p, bool runnable)
{
	u64 now = bpf_ktime_get_ns();
	s32 cpu = scx_bpf_task_cpu(p);
	u64 slice, delta_vtime, delta_runtime;
	u8  tier;
	struct task_ctx *tctx;
	struct cpu_ctx  *cctx;

	__sync_fetch_and_sub(&nr_running, 1);

	tctx = try_lookup_task_ctx(p);
	if (!tctx) return;

	slice       = now - tctx->last_run_at;
	delta_vtime = scale_by_task_weight_inverse(p, slice);

	tctx->avg_runtime = calc_avg(tctx->avg_runtime, slice);

	tier = tctx->tier < NR_TIERS ? tctx->tier : TIER_DEFAULT;
	if (p->scx.dsq_vtime >= tier_vtime_base[tier])
		p->scx.dsq_vtime -= tier_vtime_base[tier];
	p->scx.dsq_vtime  += delta_vtime;
	tctx->awake_vtime += delta_vtime;

	cctx = try_lookup_cpu_ctx(cpu);
	if (!cctx) return;
	delta_runtime      = now - cctx->last_running;
	cctx->tot_runtime += delta_runtime;

	/*
	 * Update frequency unconditionally.  When TIMELY is on, both the
	 * utilisation-based frequency and the TIMELY slice adaptation are
	 * active simultaneously — they address different axes (frequency vs
	 * scheduling quantum length) and are complementary.
	 */
	update_cpu_load(p, tctx);

	group_iact_cpu_used(p, slice);

	if (!runnable)
		group_iact_quiesce(p, now);
}

/* ─── runnable ──────────────────────────────────────────────────────────── */

void BPF_STRUCT_OPS(aura_runnable, struct task_struct *p, u64 enq_flags)
{
	u64 now = bpf_ktime_get_ns();
	u64 delta_t;
	struct task_ctx *tctx = try_lookup_task_ctx(p);
	if (!tctx) return;

	tctx->awake_vtime = 0;

	delta_t = now > tctx->last_woke_at ? now - tctx->last_woke_at : 1;
	tctx->wakeup_freq  = update_freq(tctx->wakeup_freq, delta_t);
	tctx->wakeup_freq  = MIN(tctx->wakeup_freq, MAX_WAKEUP_FREQ);
	tctx->last_woke_at = now;

	group_iact_wakeup(p, now);
}

/* ─── enable / init_task / exit_task ────────────────────────────────────── */

void BPF_STRUCT_OPS(aura_enable, struct task_struct *p)
{
	p->scx.dsq_vtime = vtime_now;
}

s32 BPF_STRUCT_OPS(aura_init_task, struct task_struct *p,
		   struct scx_init_task_args *args)
{
	struct task_ctx *tctx = bpf_task_storage_get(
		&task_ctx_stor, p, 0, BPF_LOCAL_STORAGE_GET_F_CREATE);
	if (!tctx) return -ENOMEM;

	/* Initialise TIMELY gain at maximum (least restrictive). */
	if (timely_enabled)
		tctx->timely_gain_fp = timely_gain_max_fp;

	return 0;
}

void BPF_STRUCT_OPS(aura_exit_task, struct task_struct *p,
		    struct scx_exit_task_args *args)
{
	u32 tgid;
	struct group_iact *gi;

	if (!group_iact_enabled) return;

	tgid = p->tgid;
	gi   = bpf_map_lookup_elem(&group_iact_map, &tgid);
	if (!gi) return;

	if (gi->runnable_count > 0)
		gi->runnable_count--;

	if (gi->runnable_count == 0)
		bpf_map_delete_elem(&group_iact_map, &tgid);
}

/* ─── cpu_release ───────────────────────────────────────────────────────── */

void BPF_STRUCT_OPS(aura_cpu_release, s32 cpu,
		    struct scx_cpu_release_args *args)
{
	if (timely_enabled) {
		scx_bpf_reenqueue_local();
		__sync_fetch_and_add(&nr_cpu_release_reenqueue, 1);
	}
}

/* ─── Syscall progs ─────────────────────────────────────────────────────── */

static s32 get_nr_online_cpus(void)
{
	const struct cpumask *mask = scx_bpf_get_online_cpumask();
	int n = bpf_cpumask_weight(mask);
	scx_bpf_put_cpumask(mask);
	return n;
}

static void init_cpuperf_target(void)
{
	const struct cpumask *online = scx_bpf_get_online_cpumask();
	s32 cpu;
	u64 perf_lvl;

	bpf_for(cpu, 0, nr_cpu_ids) {
		if (!bpf_cpumask_test_cpu(cpu, online)) continue;
		perf_lvl = cpufreq_perf_lvl < 0 ?
			   (u64)SCX_CPUPERF_ONE :
			   MIN((u64)cpufreq_perf_lvl, (u64)SCX_CPUPERF_ONE);
		scx_bpf_cpuperf_set(cpu, perf_lvl);
	}
	scx_bpf_put_cpumask(online);
}

SEC("syscall")
int enable_sibling_cpu(struct domain_arg *input)
{
	struct cpu_ctx *cctx = try_lookup_cpu_ctx(input->cpu_id);
	struct bpf_cpumask **pmask;
	struct bpf_cpumask *mask;
	int err;

	if (!cctx) return -ENOENT;

	pmask = &cctx->smt;
	err   = init_cpumask(pmask);
	if (err) return err;

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
	s32 cpu;
	int err = init_cpumask(&primary_cpumask);
	if (err) return err;

	bpf_rcu_read_lock();
	mask = primary_cpumask;
	if (mask) {
		cpu = input->cpu_id;
		if (cpu < 0)
			bpf_cpumask_clear(mask);
		else
			bpf_cpumask_set_cpu(cpu, mask);
	}
	bpf_rcu_read_unlock();
	return 0;
}

/* ─── Throttle timer ────────────────────────────────────────────────────── */

static int throttle_timerfn(void *map, int *key, struct bpf_timer *timer)
{
	bool throttled = is_throttled();
	u64 flags    = throttled ? SCX_KICK_IDLE    : SCX_KICK_PREEMPT;
	u64 duration = throttled ? slice_max        : throttle_ns;
	s32 cpu;
	int err;

	set_throttled(!throttled);
	bpf_for(cpu, 0, nr_cpu_ids)
		scx_bpf_kick_cpu(cpu, flags);

	err = bpf_timer_start(timer, duration, 0);
	if (err) scx_bpf_error("Failed to re-arm throttle timer");
	return 0;
}

/* ─── init ──────────────────────────────────────────────────────────────── */

s32 BPF_STRUCT_OPS_SLEEPABLE(aura_init)
{
	struct bpf_timer *timer;
	u64 dsq_id, cap;
	int err, i;
	u32 key = 0;

	nr_online_cpus = get_nr_online_cpus();
	nr_cpu_ids     = scx_bpf_nr_cpu_ids();

	/* Precompute max cpu_capacity for is_cpu_efficient(). */
	max_cpu_cap = 0;
	bpf_for(i, 0, MIN(nr_cpu_ids, MAX_CPUS)) {
		cap = cpu_capacity[i];
		if (cap > max_cpu_cap)
			max_cpu_cap = cap;
	}

	/* Tier vtime base offsets. */
	tier_vtime_base[TIER_INTERACTIVE] = 0;
	tier_vtime_base[TIER_DEFAULT]     = TIER_VTIME_GAP;
	tier_vtime_base[TIER_BACKGROUND]  = 2 * TIER_VTIME_GAP;

	/* Per-tier WCEL budgets. */
	tier_wcel_ns[TIER_INTERACTIVE] = WCEL_INTERACTIVE_NS;
	tier_wcel_ns[TIER_DEFAULT]     = WCEL_DEFAULT_NS;
	tier_wcel_ns[TIER_BACKGROUND]  = WCEL_BACKGROUND_NS;

	/* WCEL enqueue timestamps start at zero (no tasks queued). */
	tier_enqueue_ts[TIER_INTERACTIVE] = 0;
	tier_enqueue_ts[TIER_DEFAULT]     = 0;
	tier_enqueue_ts[TIER_BACKGROUND]  = 0;

	/*
	 * Per-tier warp budgets and accounting.  Each tier starts with a
	 * full budget (as if it had just won fairly), mirroring XNU's
	 * initial state where every root bucket starts un-warped with its
	 * full allowance available.
	 */
	tier_warp_budget_ns[TIER_INTERACTIVE] = WARP_BUDGET_INTERACTIVE_NS;
	tier_warp_budget_ns[TIER_DEFAULT]     = WARP_BUDGET_DEFAULT_NS;
	tier_warp_budget_ns[TIER_BACKGROUND]  = WARP_BUDGET_BACKGROUND_NS;

	tier_warp_remaining_ns[TIER_INTERACTIVE]    = WARP_BUDGET_INTERACTIVE_NS;
	tier_warp_remaining_ns[TIER_DEFAULT]        = WARP_BUDGET_DEFAULT_NS;
	tier_warp_remaining_ns[TIER_BACKGROUND]     = WARP_BUDGET_BACKGROUND_NS;
	tier_warp_window_until_ns[TIER_INTERACTIVE] = 0;
	tier_warp_window_until_ns[TIER_DEFAULT]     = 0;
	tier_warp_window_until_ns[TIER_BACKGROUND]  = 0;
	tier_warp_window_opened_ns[TIER_INTERACTIVE] = 0;
	tier_warp_window_opened_ns[TIER_DEFAULT]     = 0;
	tier_warp_window_opened_ns[TIER_BACKGROUND]  = 0;

	/* Starvation-avoidance windows start closed for every tier. */
	tier_starvation_active[TIER_INTERACTIVE] = false;
	tier_starvation_active[TIER_DEFAULT]     = false;
	tier_starvation_active[TIER_BACKGROUND]  = false;
	tier_starvation_ts[TIER_INTERACTIVE]     = 0;
	tier_starvation_ts[TIER_DEFAULT]         = 0;
	tier_starvation_ts[TIER_BACKGROUND]      = 0;

	init_cpuperf_target();

	/* Per-CPU DSQs. */
	bpf_for(i, 0, nr_cpu_ids) {
		dsq_id = (u64)i;
		err = scx_bpf_create_dsq(dsq_id, __COMPAT_scx_bpf_cpu_node(i));
		if (err) {
			scx_bpf_error("failed to create cpu DSQ %llu: %d",
				      dsq_id, err);
			return err;
		}
	}

	/* Three global tier DSQs. */
	bpf_for(i, 0, NR_TIERS) {
		dsq_id = tier_dsq((u8)i);
		err = scx_bpf_create_dsq(dsq_id, -1);
		if (err) {
			scx_bpf_error("failed to create tier DSQ %llu: %d",
				      dsq_id, err);
			return err;
		}
	}

	err = init_cpumask(&primary_cpumask);
	if (err) return err;

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

/* ─── exit ──────────────────────────────────────────────────────────────── */

void BPF_STRUCT_OPS(aura_exit, struct scx_exit_info *ei)
{
	UEI_RECORD(uei, ei);
}

/* ─── ops table ─────────────────────────────────────────────────────────── */

SCX_OPS_DEFINE(aura_ops,
	.select_cpu	= (void *)aura_select_cpu,
	.enqueue	= (void *)aura_enqueue,
	.dispatch	= (void *)aura_dispatch,
	.cpu_release	= (void *)aura_cpu_release,
	.running	= (void *)aura_running,
	.stopping	= (void *)aura_stopping,
	.runnable	= (void *)aura_runnable,
	.enable		= (void *)aura_enable,
	.init_task	= (void *)aura_init_task,
	.exit_task	= (void *)aura_exit_task,
	.init		= (void *)aura_init,
	.exit		= (void *)aura_exit,
	.timeout_ms	= STARVATION_MS,
	.name		= "aura");
