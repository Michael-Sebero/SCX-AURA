## **A.U.R.A**

**Adaptive · Utilisation · Responsive · Architecture**

> **ABSTRACT**: `scx_aura` is a BPF CPU scheduler built on [sched_ext](https://github.com/sched-ext/scx) for **laptop workloads** that need both low-latency responsiveness and long battery life. It sorts every task into one of four QoS tiers modelled on Apple XNU's Clutch scheduler, picks which tier runs next with XNU's root-bucket algorithm, places work on performance or efficiency cores according to its tier and drives CPU frequency from frequency-invariant utilisation.
>
> - **4-Tier QoS** Interactive / Default / Utility / Background, mapped to XNU's FG / DF / UT / BG root buckets
> - **Clutch Root-Bucket Selection** Earliest-deadline-first across tiers using XNU's worst-case execution latencies, depleting warp budgets and one-quantum starvation-avoidance windows, evaluated in XNU's order
> - **Behavioural Classification** CPU time per wakeup, wakeup rate and an XNU thread-group interactivity score, with hysteresis so tasks near a threshold do not flip tiers
> - **Interactive Preemption** A waking interactive task that finds no idle CPU takes the CPU running the lowest tier, big cores first; the preempted task resumes there with only its remaining slice
> - **AMP Placement** Background work prefers efficiency cores and is confined to them in powersave mode; misplaced work moves or swaps cores at quantum expiry, as in XNU's Edge scheduler
> - **CLPC-Style Frequency Control** With schedutil, per-CPU frequency targets come from utilisation, with a floor for interactive work and caps for utility and background work
> - **Live Power Modes** Performance / balanced / powersave, following power-profiles-daemon or the energy performance preference, applied without restarting the scheduler

## Navigation

- [1. Quick Start](#1-quick-start)
- [2. Philosophy](#2-philosophy)
- [3. Tiers](#3-tiers)
- [4. Root-Bucket Selection](#4-root-bucket-selection)
- [5. Dispatch, Preemption and Resume](#5-dispatch-preemption-and-resume)
- [6. Core Placement](#6-core-placement)
- [7. Power Management](#7-power-management)
- [8. Adaptive Time Slices (TIMELY)](#8-adaptive-time-slices-timely)
- [9. Architecture](#9-architecture)
- [10. Options](#10-options)
- [11. Statistics](#11-statistics)
- [12. Overhead](#12-overhead)
- [13. Vocabulary](#13-vocabulary)

---

## 1. Quick Start

Requirements: a kernel with sched_ext (`CONFIG_SCHED_CLASS_EXT=y`) and BTF, a Rust toolchain and clang. Tested on Linux 6.17 and 7.0 with BPF built by clang 18 and clang 20.

```bash
git clone https://github.com/Michael-Sebero/SCX-AURA
cd SCX-AURA && cargo build --release
sudo install -m 755 target/release/scx_aura /bin/scx_aura
```

```bash
# Run with the laptop preset (requires root)
sudo scx_aura

# Force a power mode instead of following the system profile
sudo scx_aura --power-mode powersave

# Hybrid CPU whose efficiency cores are not detected: CPUs 8-15 are E-cores
sudo scx_aura --little-cpus 0xff00

# Adaptive time slices
sudo scx_aura --timely

# Live statistics, one line per second
sudo scx_aura --stats 1
```

Check that it is active:

```bash
cat /sys/kernel/sched_ext/state /sys/kernel/sched_ext/root/ops
```

The scheduler runs in the foreground for as long as it is loaded. To start it at boot from an rc script, detach it:

```bash
( setsid nohup scx_aura >> /var/log/scx_aura.log 2>&1 & )
```

---

## 2. Philosophy

Fair schedulers (CFS, EEVDF) give a browser and a compiler roughly equal CPU time. On a laptop this causes two problems:

1. **Latency inversion**: a 50 µs UI callback waits behind a compile job's slice
2. **Power waste**: background work runs on performance cores at high frequency and work that could wait keeps big cores out of deep idle

XNU solves this with QoS classes that applications declare. Linux has no QoS classes, so scx_aura infers them: explicit hints first (nice, `SCHED_BATCH`, `SCHED_IDLE`), then behaviour. Tasks that burst briefly and wake often are interactive; long-running tasks in CPU-bound process groups are utility; nice ≥ 10 and `SCHED_IDLE` are background. Each tier then gets XNU's treatment: a latency bound, a core class and a frequency policy. No cgroup setup, taskset or per-application profiles are needed.

---

## 3. Tiers

### Tier Table

Latency bounds, warp budgets and starvation windows are XNU's values scaled by `slice_max / 10 ms`. The table shows them at the default 800 µs slice.

| Tier | XNU bucket | Worst-case latency | Warp | Starvation window | Examples |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Interactive** | FG | 0 ms | 640 µs | 800 µs | Input handlers, audio callbacks, UI threads |
| **Default** | DF | 6 ms | 160 µs | 800 µs | Shells, browsers, video playback |
| **Utility** | UT | 12 ms | 80 µs | 320 µs | Compilers, encoders, `SCHED_BATCH` jobs |
| **Background** | BG | 20 ms | 0 | 160 µs | nice ≥ 10, `SCHED_IDLE`, indexers |

XNU's unscaled values (10 ms quantum): latency 0 / 75 / 150 / 250 ms, warp 8 / 2 / 1 / 0 ms, quantum 10 / 10 / 4 / 2 ms.

### Classification

Tiers are re-evaluated at every wakeup, enqueue and slice refill. The first matching rule wins:

1. `SCHED_IDLE` or nice ≥ 10 → **Background**. On a hybrid CPU, a task whose affinity excludes every efficiency core gets **Utility** instead.
2. nice < −5 → **Interactive**
3. `SCHED_BATCH` → **Utility**
4. **Interactive** when the task's burst is at most `slice_max / 2` (400 µs) and either it wakes at least 16 times per 100 ms, or its process group's interactivity score is at least 12/16
5. **Utility** when the group score is at most 4/16, the burst is at least `slice_max` and the task wakes fewer than 2 times per 100 ms
6. Otherwise **Default**

Staying in a behavioural tier takes looser thresholds than entering it:

| Tier | Enter | Stay |
| :--- | :--- | :--- |
| Interactive | burst ≤ 400 µs, wakeups ≥ 16/100 ms or score ≥ 12 | burst ≤ 800 µs, wakeups ≥ 8/100 ms or score ≥ 10 |
| Utility | burst ≥ 800 µs, wakeups < 2/100 ms, score ≤ 4 | burst ≥ 400 µs, wakeups < 4/100 ms, score ≤ 6 |

### Bursts and Wakeup Rate

A **burst** is the CPU time a task uses between two wakeups. Being preempted does not end a burst, so a CPU-bound task never looks bursty however often it is interrupted. Classification uses the larger of the burst average (EWMA, α = 1/4) and the burst in progress. New tasks start with an average of one full slice, so they become interactive only after their wakeups show short bursts.

The wakeup rate is an EWMA of wakeups per 100 ms, updated at each wakeup and capped at 64. Because it only changes on wakeups, it is also bounded by `100 ms / burst in progress`: a task that stops sleeping loses its wakeup credit instead of keeping it forever.

### Thread-Group Interactivity Score

Each process (by tgid) accumulates CPU-used and blocked time, where blocked means no thread of the process is runnable. The score is XNU's `sched_clutch_interactivity_from_cpu_data()`:

- `blocked > used`: `score = 8 + 8 × (blocked − used) / blocked` (8–16, interactive)
- otherwise: `score = 8 × blocked / used` (0–8, CPU-bound)

When used + blocked reaches 500 ms, both are divided by 10, so only recent behaviour counts. As in XNU, a new process starts fully interactive (500 ms blocked, score 16) and moves toward its real behaviour as it runs. Scores live in an 8192-entry LRU hash keyed by tgid; the group leader's start time detects tgid reuse.

---

## 4. Root-Bucket Selection

Each tier has a shared dispatch queue ordered by bpfland's virtual deadline. Which tier a CPU consumes from next is chosen by XNU's `sched_clutch_root_highest_root_bucket()`, over the tiers that have queued work or that the CPU's current task belongs to.

Each tier has a **deadline**: the time it became runnable or was last selected, plus its worst-case latency. Selection runs in this order:

1. **Starvation window open**: if the earliest-deadline tier is inside a starvation-avoidance window, it is selected.
2. **Natural order**: if the earliest-deadline tier is also the highest runnable tier, it is selected. Its deadline moves out by its latency bound and its warp budget is refilled.
3. **Warp**: otherwise, the highest tier above it that has warp left is selected. The first warp selection opens a window as long as the remaining budget; the budget is spent in wall time and is gone when the window closes.
4. **Starvation avoidance**: otherwise, the earliest-deadline tier is selected and a starvation window of one tier quantum opens. When the window closes, its deadline moves out by its latency bound.

The warp budget is refilled only by a natural-order selection, so a tier with continuous arrivals cannot run ahead indefinitely. When a tier's queue empties during an open warp window, the unused part is kept for its next runnable period. All root-bucket state is global and protected by one spin lock.

Within a tier, tasks are ordered by vruntime plus the vruntime accumulated since their last wakeup, with sleep credit bounded by `slice_lag` scaled by the wakeup rate. Each tier has its own vruntime clock; a task changing tier keeps its lag relative to the clock, clamped to `±slice_lag`.

---

## 5. Dispatch, Preemption and Resume

### Slice Expiry

When the running task's slice expires, `ops.dispatch()` charges its runtime, re-classifies it and runs root-bucket selection with the task's tier included:

| Outcome | What happens to the running task |
| :--- | :--- |
| Its own tier is selected and it is within one slice of the queue head's deadline | Keeps the CPU with a new slice |
| A lower tier gets an EDF or starvation turn | Resumes on the same CPU right after that turn |
| A higher tier is selected | Goes back to its tier queue and competes again |
| Its tier is selected but another task's deadline is earlier by more than a slice | Goes back to its tier queue |

Slices are `slice_max` scaled by task weight, divided by the number of queued tasks and clamped to `[slice_min, slice_max]` (250–800 µs by default).

### Interactive Preemption

A waking interactive task first looks for an idle CPU. If there is none, it takes a CPU running a lower tier: its previous CPU if that is a big core running lower-tier work, otherwise the big core running the lowest tier, otherwise a little core. It is inserted at the head of that CPU's local queue with `SCX_ENQ_PREEMPT`.

The preempted task goes back onto that CPU's local queue behind it, with only the slice it had left, so a preemption never grants extra CPU time.

### Other Local Placements

- **Sticky tasks**: tasks averaging under 10 µs of CPU time per wakeup stay on their CPU (`--no-sticky-tasks` to disable)
- **Per-CPU tasks**: tasks pinned to one CPU or with migration disabled go straight to that CPU (`--no-local-pcpu` to disable)
- **Higher scheduling classes**: when an RT or deadline task takes a CPU, the tasks waiting in its local queue are re-enqueued so they can run elsewhere. On kernels with `scx_bpf_reenqueue_local___v2` this runs from a `sched_switch` tracepoint; older kernels use `ops.cpu_release()`.

Any task inserted into a tier queue kicks an idle CPU that could run it.

---

## 6. Core Placement

Efficiency cores are detected from the kernel's core types; `--little-cpus` overrides the detection. Placement is enabled only when both core classes are present and is disabled with `--no-qos-placement`.

| Tier | Balanced / performance | Powersave |
| :--- | :--- | :--- |
| Interactive | Big cores first | Big cores first |
| Default | Big cores first | Big cores first |
| Utility | Big cores first | Little cores first |
| Background | Little cores first | Little cores only |

### Idle CPU Selection

Performance-first tiers look for a whole idle big core, then a little core, then any idle CPU. Efficiency-first tiers look for a little core (lowest capacity first), then, unless confined, an idle SMT sibling of an already awake core, then any CPU. The previous CPU is tried first. The default scan walks CPUs in capacity order; `--no-preferred-idle-scan` uses the kernel's topology-aware `scx_bpf_select_cpu_and()` with the same core-class masks instead.

### Shared Queues

The tier queues are shared by both core classes. A big core consumes background work only when no other tier has queued work, so background tasks cannot take big cores while foreground work waits on little ones.

### Quantum-Expiry Rebalancing

At every slice expiry, a task on the wrong core class is moved, as in XNU's AMP/Edge rebalancing:

- **Up**: work that prefers big cores, running on a little core, moves to an idle whole big core. If there is none, it takes a big core that is running little-core work; the displaced task goes back to its tier queue for a little core to pick up.
- **Down**: work that prefers little cores, running on a big core, moves to an idle little core.
- **SMT**: a task whose SMT sibling is busy moves to a whole idle core of the same class.

---

## 7. Power Management

### Power Modes

`--power-mode auto` (default) follows power-profiles-daemon over D-Bus and falls back to the cpufreq energy performance preference or governor name when the daemon is not running. A profile change is applied in place within one second, without restarting the scheduler. If nothing reports a profile, auto uses balanced.

| Mode | Interactive frequency floor | Utility cap | Background cap | Background confined to little cores | Utility prefers little cores |
| :--- | :--- | :--- | :--- | :--- | :--- |
| Performance | 100% | none | none | No | No |
| Balanced | 75% | none | 50% | No | No |
| Powersave | 50% | 75% | 37.5% | Yes | Yes |

Floors and caps are fractions of each CPU's maximum frequency and apply only while a task of that tier runs on the CPU.

### Frequency Control

> [!IMPORTANT]
> Frequency control only takes effect with the **schedutil** governor: amd-pstate or intel_pstate in passive mode, or amd-pstate in guided mode. With amd-pstate or intel_pstate in active (EPP) mode the hardware picks frequencies itself and the power mode only affects placement.

Each CPU folds its busy time into a utilisation estimate every 4 ms:

- **Frequency-invariant**: busy time is weighted by the current frequency (`scx_bpf_cpuperf_cur()`), so utilisation means "fraction of this CPU at maximum frequency"
- **Fast attack**: the estimate moves halfway to a higher sample at once
- **Proportional decay**: it falls by 1/8 of the gap per elapsed 4 ms window, up to the full gap after 32 ms
- **Saturation boost**: a CPU busy for at least 7/8 of a window at its current frequency is pushed halfway toward maximum, since its real demand is unknown

The estimate is scaled by CPU capacity, because schedutil reads the target as utilisation in capacity units and the tier's floor or cap is applied. schedutil adds 25% headroom to everything it reads, so floors and caps are set at 4/5 of their nominal value to land on the frequency in the table. `--no-cpufreq` pins the target at maximum instead.

### Idle Resume Latency

Off by default. `-I <us>` limits the idle-state exit latency of the primary-domain CPUs, excluding idle states that take longer to exit. The limit is applied after the scheduler has attached and restored when it exits or fails. On current laptop CPUs the deepest core C-states exit in a few hundred microseconds, so a low limit costs battery and a high one does nothing.

---

## 8. Adaptive Time Slices (TIMELY)

Enabled with `--timely`. The structure follows TIMELY (Mittal et al., SIGCOMM 2015), with a task's queueing delay in place of network RTT. Each task has a gain in `[gain_min, 1024]` (0.125–1.0 by default) that multiplies its slice.

The queueing delay is the time from enqueue to running; its gradient is an EWMA (α = 1/4) of the change between samples. The gain is updated at most once per control interval (500 µs):

| Region | Condition | Gain update |
| :--- | :--- | :--- |
| Low delay | delay < `tlow` (5 ms) | `gain += step` (32) |
| High delay | delay > `thigh` (50 ms) | `gain *= 1 − β × (1 − thigh / delay)` |
| Falling or flat | gradient ≤ 0 | `gain += step`, or `step × hai_multiplier` (2) after `hai_thresh` (5) consecutive samples (hyperactive increase) |
| Rising | gradient > 0 | `gain *= 1 − β × min(gradient / margin, 1)` |

β defaults to 819/1024 (0.8) and the gradient margin to 125 µs. All TIMELY state is per-task.

---

## 9. Architecture

### Callbacks

```
select_cpu  re-classify, pick an idle CPU by tier and core class, direct-dispatch if found
enqueue     consume a migration claim, resume a preempted task, re-enqueue after RT,
            sticky and per-CPU placement, idle CPU, interactive preemption, tier queue + kick
dispatch    charge prev, re-classify, quantum-expiry rebalance, root-bucket selection,
            keep prev or consume a tier
running     tier clock, TIMELY sample, running-tier gauge, frequency target
stopping    charge runtime, close the CPU's utilisation interval
runnable    group runnable count and blocked time; on wakeup: burst average, wakeup rate
quiescent   group runnable count, start of blocked interval
cpu_release re-enqueue the local queue (kernels without the any-context kfunc)
```

### Data Structures

| Structure | Storage | Contents |
| :--- | :--- | :--- |
| `task_ctx` | task storage | tier, burst and average burst, wakeup rate, sleep credit, TIMELY gain and delay state |
| `cpu_ctx` | per-CPU array | utilisation window, frequency target, running tier and pid, slice end, resume and migration claims, SMT sibling mask |
| `group_ctx` | LRU hash (8192, by tgid) | leader start time, CPU used, blocked time, runnable threads, score |
| `root_bkt[4]` | global, spin lock | deadline, warp remaining, warp window, starvation window per tier |

### DSQ Layout

| ID | Purpose |
| :--- | :--- |
| `SCX_DSQ_LOCAL` / `SCX_DSQ_LOCAL_ON` | Direct dispatch, resumes, interactive preemption, migrations |
| `0x100` | Interactive tier |
| `0x101` | Default tier |
| `0x102` | Utility tier |
| `0x103` | Background tier |

---

## 10. Options

```
Scheduling
-s, --slice-us <us>                 Maximum slice; latency bounds, warp and windows scale with it (default: 800)
-L, --slice-min-us <us>             Minimum slice (default: 250)
-l, --slice-us-lag <us>             Maximum sleep credit (default: 40000)
-t, --throttle-us <us>              Inject idle cycles periodically (default: 0, disabled)
-m, --primary-domain <domain>       Hex cpumask, or auto / performance / powersave / turbo / all (default: performance)
-k, --local-kthreads                Dispatch per-CPU kthreads locally (experimental)
-w, --no-wake-sync                  Ignore synchronous wakeups (kernel idle CPU selection only)
    --disable-smt                   Disable SMT awareness
    --disable-numa                  Disable NUMA awareness

Power
    --power-mode <mode>             auto / performance / balanced / powersave (default: auto)
    --little-cpus <mask|none>       Override detected efficiency cores
-I, --idle-resume-us <us>           Idle resume latency limit on the primary domain (default: -1, disabled)
    --no-cpufreq                    Do not drive CPU frequency (pins schedutil at maximum)

Feature opt-outs
    --no-sticky-tasks               Disable sticky tasks
    --no-local-pcpu                 Queue single-CPU tasks in their tier
    --no-preferred-idle-scan        Use the kernel's idle CPU selection
    --no-group-iact                 Disable thread-group interactivity scoring
    --no-qos-placement              Disable core-type placement (alias: --no-ecore-consolidate)
    --no-warp                       Disable warp

TIMELY (only with -T)
-T, --timely                        Enable adaptive time slices
    --timely-tlow-us <us>           Low delay threshold (default: 5000)
    --timely-thigh-us <us>          High delay threshold (default: 50000)
    --timely-gain-min <fp>          Minimum gain, 1024 = 1.0 (default: 128)
    --timely-gain-step <fp>         Additive step (default: 32)
    --timely-hai-thresh <n>         Non-rising samples before hyperactive increase (default: 5)
    --timely-hai-multiplier <n>     Step multiplier in hyperactive increase (default: 2)
    --timely-beta <fp>              Multiplicative decrease factor, 819 = 0.8 (default: 819)
    --timely-gradient-margin-us <us> Gradient normalisation (default: 125)
    --timely-control-interval-us <us> Minimum interval between gain updates (default: 500)

Diagnostics
    --stats <sec>                   Run with live statistics
    --monitor <sec>                 Statistics only, scheduler not launched
    --help-stats                    Describe the statistics fields
    --exit-dump-len <n>             Exit dump buffer length (default: 0, kernel default)
-d, --debug                         BPF debug output to /sys/kernel/tracing/trace_pipe
-v, --verbose                       Verbose output including libbpf
-V, --version                       Print version
```

---

## 11. Statistics

`--stats 1` prints one line per interval. Counters are per interval; `r` and `run` are sampled.

```
[scx_aura] r: 4/4  bal run i0 d1 u2 b1 | enq i:12 d:48 u:6 b:24 | disp ... | sel nat:1305 warp:3 starve:2/0 | kick:13 mig ^2 v0 smt:0 | tier ^4 v4 | keep:1223 direct:9 local:31
```

| Field | Meaning |
| :--- | :--- |
| `r: 4/4` | Running tasks / online CPUs |
| `bal` | Active power mode (`perf`, `bal`, `save`) |
| `run i d u b` | CPUs running each tier at sample time (includes the stats reader itself) |
| `enq`, `disp` | Tier queue enqueues and dispatches per tier |
| `sel nat warp starve:o/s` | Root-bucket selections: natural order, warp, starvation windows opened / selections inside one |
| `kick` | Interactive preemptions |
| `mig ^ v smt` | Quantum-expiry moves up to big cores (including swaps), down to little cores, away from busy SMT siblings |
| `tier ^ v` | Tier promotions and demotions |
| `keep`, `direct`, `local` | Slice refills, direct placements on idle CPUs, local-queue inserts (sticky, per-CPU, resumes) |

With `--timely`, a second line counts additive increases, multiplicative decreases and hyperactive increases.

---

## 12. Overhead

BPF instruction counts with clang 20 at `-O2`:

| Program | Instructions |
| :--- | :--- |
| `dispatch` | 1361 |
| `enqueue` | 841 |
| `running` | 249 |
| `runnable` | 166 |
| `select_cpu` | 84 |
| `stopping` | 61 |
| `quiescent` | 48 |
| Shared subprograms | 1279 |

- **Root-bucket selection**: takes one global spin lock per dispatch, plus one when a tier becomes runnable. Four DSQ length reads per dispatch decide which tiers are candidates.
- **Group scoring**: one hash lookup per classification and per runtime charge.
- **CPU scans**: preemption target selection and big/little swaps scan at most `nr_cpus` entries and only when no idle CPU is available. Swaps run only on hybrid CPUs.
- **Frequency**: `scx_bpf_cpuperf_set()` is called only when the target changes.

---

## 13. Vocabulary

### Scheduling

| Term | Definition |
| :--- | :--- |
| **Tier** | QoS class (Interactive, Default, Utility, Background); decides root bucket, core class and frequency policy |
| **Root bucket** | XNU's per-QoS scheduling entity; here, one per tier with a deadline, warp budget and starvation window |
| **WCEL** | Worst-case execution latency: how long a runnable tier may wait before its deadline expires |
| **Warp** | Budget that lets a higher tier run ahead of the earliest-deadline tier; spent in wall time, refilled only by a natural-order selection |
| **Starvation-avoidance window** | One tier quantum during which an expired lower tier is selected even though a higher tier is runnable |
| **Burst** | CPU time a task uses between two wakeups |
| **Wakeup rate** | EWMA of wakeups per 100 ms |
| **Interactivity score** | Per-process 0–16 value from blocked versus used CPU time; 16 is fully interactive |
| **Lag** | Sleep credit that pulls a task's virtual deadline earlier, bounded by `slice_lag` |
| **Resume** | A task that lost its CPU to a preemption or a lower tier's turn continues on the same CPU right after |
| **Gain** | TIMELY slice multiplier in `[gain_min, 1024]` |

### Hardware

| Term | Definition |
| :--- | :--- |
| **Big / P-core** | Performance core: higher capacity and power draw |
| **Little / E-core** | Efficiency core: lower capacity and power draw |
| **SMT** | Two logical CPUs per physical core; whole idle cores are preferred |
| **EPP** | Energy performance preference, the cpufreq hint used by `--power-mode auto` without power-profiles-daemon |
| **schedutil** | cpufreq governor that reads the scheduler's frequency target |

### Research Sources

| Feature | Derived from |
| :--- | :--- |
| Virtual deadline with sleep credit, idle selection | scx_bpfland |
| Root-bucket EDF, warp, starvation avoidance | XNU `sched_clutch.c`, `sched_clutch_root_highest_root_bucket()` |
| Thread-group interactivity score and decay | XNU `sched_clutch_interactivity_from_cpu_data()` |
| Core-class placement and quantum-expiry rebalancing | XNU AMP and Edge schedulers (`sched_amp.c`) |
| QoS-based frequency floors and caps | Apple CLPC behaviour, via schedutil |
| Delay-driven slice gain | TIMELY (Mittal et al., SIGCOMM 2015) |
