## **A.U.R.A**

**Adaptive · Utilisation · Responsive · Architecture**

> **ABSTRACT**: `scx_aura` is a BPF CPU scheduler built on [sched_ext](https://github.com/sched-ext/scx), designed for **laptop workloads** that demand both low-latency responsiveness and long battery life. It classifies every task by observed sleep and CPU behaviour, routes work through a 3-tier priority system, and actively manages CPU placement and frequency to keep efficiency cores idle.
>
> - **3-Tier Classification** Tasks sorted into Interactive / Default / Background by per-process sleep-to-CPU ratio and per-task wakeup frequency
> - **Starvation Enforcement** Each tier has a hard latency budget (0 ms / 75 ms / 250 ms); expired budgets win dispatch unconditionally regardless of normal ordering
> - **Priority Dispatch Window** Interactive tasks arm an 8 ms priority window on arrival; during that window they preempt all lower-tier work without waiting for EDF ordering
> - **Adaptive Time-Slice Feedback** Per-task queue delay drives a fixed-point gain that scales each task's slice up or down; gradient detection and a high-activity-index streak catch rising and sustained congestion
> - **E-Core Idle Consolidation** When the system is below 75% load, all tiers avoid efficiency cores, letting them reach deep C-states and extend battery life
> - **Per-Process Interactivity Scoring** Each process group accumulates CPU-used and voluntary-sleep time; the ratio updates a 0–16 score that drives tier placement and decays over time so past behaviour does not permanently define a group

## Navigation

- [1. Quick Start](#1-quick-start)
- [2. Philosophy](#2-philosophy)
- [3. 3-Tier System](#3-3-tier-system)
- [4. Priority Dispatch Window](#4-priority-dispatch-window)
- [5. Adaptive Time-Slice Feedback](#5-adaptive-time-slice-feedback)
- [6. Power Management](#6-power-management)
- [7. Architecture](#7-architecture)
- [8. Default Preset](#8-default-preset)
- [9. Options](#9-options)
- [10. Overhead](#10-overhead)
- [11. Vocabulary](#11-vocabulary)

---

## 1. Quick Start

```bash
# Prerequisites: Linux Kernel 6.12+ with sched_ext, Rust toolchain

# Clone and build
git clone https://github.com/Michael-Sebero/SCX-AURA
cd SCX-AURA && cargo build --release

# Install
sudo mv target/release/scx_aura /bin/
chmod 755 /bin/scx_aura

# Run (requires root) — loads the laptop preset automatically
sudo scx_aura

# Performance cores only, no frequency scaling
sudo scx_aura -m performance --no-cpufreq

# With adaptive time-slice feedback enabled
sudo scx_aura --timely

# Monitor live statistics
sudo scx_aura --stats 1
```

---

## 2. Philosophy

Traditional schedulers (CFS, EEVDF) optimise for **fairness** — if a browser and a compiler both run, each gets roughly 50% CPU time. For interactive laptop use, this creates two problems:

1. **Latency inversion**: A 50 µs UI callback waits behind a 50 ms compile job
2. **Power waste**: Waking an efficiency core for a brief interactive task prevents that core from reaching deep idle, burning power for no throughput gain

**scx_aura's answer**: Classify tasks by *behaviour* (how long they sleep versus how much CPU they use), not by type or nice value. Interactive tasks — those that sleep often and burst briefly — get priority dispatch and P-core placement. CPU-bound tasks get larger slices but lower priority and tolerate E-core placement. The system self-tunes: no manual cgroup setup, no taskset, no explicit game-mode profiles required.

---

## 3. 3-Tier System

Every task is classified into one of three tiers. Classification is continuous — tasks move between tiers as their behaviour changes.

### Tier Table

| Tier | Name | Criteria | Budget | Examples |
| :--- | :--- | :--- | :--- | :--- |
| **T0** | Interactive | nice < −5, wakeup\_freq ≥ 16/100ms, or group score ≥ 12/16 | 0 ms (immediate) | Audio callbacks, UI threads, input handlers |
| **T1** | Default | Everything else | 75 ms | Browser tabs, shell commands, video playback |
| **T2** | Background | nice ≥ 10, or group score ≤ 4/16 | 250 ms | Compilers, package managers, background indexing |

T0 always runs before T1, which always runs before T2. This ordering is encoded in the virtual runtime key — lower tiers receive a fixed vtime offset of 400 ms per tier step, making cross-tier comparisons a plain `u64` less-than with no per-dispatch branching.

### Classification Priority

Tier assignment follows a strict priority chain for each task:

1. **Nice hard limits** — `nice < −5` forces T0; `nice ≥ 10` forces T2. These override all other signals.
2. **Per-task wakeup frequency** — tasks waking ≥ 16 times per 100 ms are placed in T0 regardless of process history. Catches audio and input threads in processes that also run CPU-bound work.
3. **Per-process interactivity score** — each process group (by tgid) accumulates CPU-used time and voluntary sleep time. The score formula is:
   - When `blocked ≥ used`: `score = 8 + 8 × (blocked − used) / blocked` (range 8–16, interactive)
   - When `blocked < used`: `score = 8 × blocked / used` (range 0–8, CPU-bound)
   - New groups start at score 16 (fully interactive) and decay toward their actual behaviour over 500 ms windows
   - Score ≥ 12 → T0; score ≤ 4 → T2; otherwise T1

> [!TIP]
> **No browser tab should stay in T2.** A tab that is actively rendering will wake frequently (T0 by wakeup frequency) or have a high sleep ratio (T0 by score). A tab that is genuinely idle will have a very low wakeup rate and a near-zero CPU-used accumulator, keeping its score high and its tier at T0 when it does wake. Only a tab doing sustained JS computation without sleeping (rare) will land in T1.

### Score Decay

The per-process score is recalculated every time the process's on-CPU time crosses a 500 ms window boundary. At that point both accumulators (CPU-used and blocked-accum) are divided by 10 and the window generation counter advances. This means a process that was CPU-bound for 500 ms but then becomes interactive recovers its score within the next 500 ms window — old behaviour does not permanently suppress the tier.

---

## 4. Priority Dispatch Window

When a T0 task is enqueued into a tier DSQ (no idle CPU was available for direct dispatch), it arms an 8 ms per-CPU priority window. During that window, every dispatch call on every CPU drains the interactive DSQ first, bypassing normal EDF ordering.

| Property | Value |
| :--- | :--- |
| Window duration | 8 ms |
| Scope | Per-CPU (each CPU tracks its own deadline) |
| Trigger | Any T0 enqueue to the tier DSQ |
| Effect | T0 tasks are consumed ahead of all other tiers regardless of virtual deadline |
| Fallback | After 8 ms or if the interactive DSQ empties, normal EDF resumes |

When a T0 task is enqueued, every busy CPU is kicked with `SCX_KICK_PREEMPT` and every idle CPU is woken with `SCX_KICK_IDLE`. Each CPU that receives the kick arms its own priority window in the dispatch path when it sees pending interactive work, so the window propagates to the CPU that will actually run the task rather than being a global lock.

### Starvation Enforcement

Independent of the priority window, each tier has a hard latency budget enforced by tracking the wall-clock timestamp of when the oldest task entered each tier DSQ. If that age exceeds the tier's budget, the dispatch path substitutes a synthetic zero-deadline for that tier's head task during the EDF selection, making it win unconditionally.

| Tier | Budget |
| :--- | :--- |
| Interactive | 0 ms (always wins) |
| Default | 75 ms |
| Background | 250 ms |

The starvation timestamp only clears when the tier DSQ becomes empty, so the budget correctly tracks the age of the queue's oldest waiting task, not just the most recently dispatched one.

---

## 5. Adaptive Time-Slice Feedback

Enabled with `--timely`. Each task maintains a fixed-point gain value (`gain_fp`, range 128–1024, representing 0.125×–1.0× of `slice_max`) that scales its slice. The gain is updated once per `control_interval_ns` (default 500 µs) based on measured queue delay.

### Three-Region Control

| Delay region | Condition | Action |
| :--- | :--- | :--- |
| Low | delay < `tlow_ns` (5 ms) | Gain += `gain_step` (32). Reset HAI streak. Slices grow: less preemption overhead. |
| Mid — rising | `tlow` ≤ delay ≤ `thigh`, gradient > `margin` | Gain × `backoff_gradient` (0.969×). Mild backoff before congestion peaks. |
| Mid — falling | `tlow` ≤ delay ≤ `thigh`, gradient < −`margin` | If gain ≥ recovery floor, gain += `gain_step/2`. Controlled recovery. |
| High | delay > `thigh_ns` (50 ms) | Gain × `backoff_high` (0.937×). Slices shrink: more preemption, better fairness. |

### High-Activity-Index (HAI) Streak

When gain drops below `hai_thresh` (768, or 0.75×) due to sustained high delay, a streak counter increments each control interval. When the streak reaches `hai_multiplier` (2), the gain is halved and the streak resets. This catches persistent congestion that the per-interval backoff alone would only gradually resolve.

### Delay Measurement

Queue delay = time from `timely_last_enqueued_at` (set at enqueue) to the moment the task begins running. Both a delay EWMA (α = 1/4) and a gradient EWMA track the signal. All TIMELY state is per-task and adds no contention between tasks on different CPUs.

---

## 6. Power Management

### E-Core Idle Consolidation

When `nr_running < 75% × nr_online_cpus` (lightly loaded), tasks of **all tiers** avoid efficiency cores during idle CPU selection. If the only available idle CPU is an E-core, the task queues to the tier DSQ rather than waking that core. This allows E-cores to remain in deep C-states (C6 on Intel ~130 µs exit latency, CC6 on AMD ~150 µs) during periods of light activity.

The threshold is intentional — at 75% load the system is no longer lightly loaded and E-core avoidance stops, so there is no throughput cost under sustained workloads.

### CPU Frequency Scaling

The scheduler drives per-CPU frequency via `scx_bpf_cpuperf_set` based on measured utilisation:

```
utilisation = (on_cpu_ns / elapsed_ns) × SCX_CPUPERF_ONE
```

Utilisation ≥ 75% snaps to maximum frequency. Below that, frequency tracks utilisation proportionally. A cold-start guard prevents the first measurement (which would divide by uptime) from spuriously driving frequency to zero.

When `--timely` is enabled, both mechanisms are active simultaneously: frequency scales the CPU's absolute speed; the TIMELY gain scales the scheduling quantum length. They address different axes and do not interfere.

### Idle Resume Latency

By default, a 1000 µs PM QoS latency constraint is applied to every CPU. This permits deep C-states whose exit latency is below 1 ms (C6/CC6 on most current laptop silicon) while blocking pathological deep states (C10, PC10) with exit latencies of 500 µs–2 ms that add visible latency to interactive events. Restored to hardware default on scheduler exit.

---

## 7. Architecture

### Hook Sequence

```
select_cpu → sample enqueue_ts, classify tier, pick idle CPU, direct-dispatch if idle
enqueue    → sample enqueue_ts, tier DSQ insert, arm priority window if T0, kick CPUs
dispatch   → warp window check → starvation check → EDF across all DSQs → keep_running
running    → sample delay, update TIMELY gain, advance vtime_now from raw vtime
stopping   → advance vruntime, update cpufreq utilisation, update group score
runnable   → update wakeup_freq EWMA, accumulate group blocked time
exit_task  → delete group score map entry when last thread exits
```

### Key Data Structures

| Structure | Purpose |
| :--- | :--- |
| `task_ctx` | Per-task: vruntime, wakeup\_freq, avg\_runtime, tier, TIMELY gain/delay/gradient/HAI |
| `cpu_ctx` | Per-CPU: runtime accumulators, frequency tracking, per-CPU warp deadline |
| `group_iact` | Per-tgid: cpu\_used\_ns, blocked\_accum\_ns, score, tier, decay generation counter |

### DSQ Layout

| ID range | Purpose |
| :--- | :--- |
| `[0 .. nr_cpu_ids)` | Per-CPU DSQs for direct dispatch (sticky, kthread, pcpu, idle-found paths) |
| `nr_cpu_ids + 0` | TIER\_INTERACTIVE global DSQ |
| `nr_cpu_ids + 1` | TIER\_DEFAULT global DSQ |
| `nr_cpu_ids + 2` | TIER\_BACKGROUND global DSQ |

Cross-tier EDF ordering is a plain `u64` comparison: `tier_vtime_base` offsets (0 / 400 ms / 800 ms) are baked into `dsq_vtime` at enqueue and stripped at stopping, so no per-dispatch branching is needed to enforce tier priority.

---

## 8. Default Preset

Running `scx_aura` with no arguments loads a laptop-optimised preset. Every value is individually overridable.

| Setting | Preset value | Upstream default | Reason |
| :--- | :--- | :--- | :--- |
| Max slice | 800 µs | 1000 µs | More scheduling opportunities for interactive tasks |
| Primary domain | Performance cores | auto | P-cores preferred on hybrid CPUs, works with E-core consolidation |
| Sticky tasks | On | Off | Short-runtime tasks stay on warm cache |
| CPU frequency | Auto (utilisation-based) | Off | Frequency tracks load without governor |
| Idle resume latency | 1000 µs | Disabled | Permits C6, blocks C10 |
| Preferred idle scan | On | Off | Capacity-sorted idle selection for deterministic P-core preference |
| Group interactivity | On | — | Per-process scoring active |
| Priority dispatch window | On | — | 8 ms T0 priority window active |
| E-core consolidation | On | — | Active below 75% system load |
| Adaptive slices (TIMELY) | Off | Off | Opt-in with `--timely` |

---

## 9. Options

```
-s <us>                     Maximum time slice (default: 800 µs)
-L <us>                     Minimum time slice (default: 0, disabled)
-l <us>                     Slice lag window (default: 40000 µs)
-m <domain>                 Primary CPU domain: auto / performance / powersave / turbo / bitmask
-I <us>                     Idle resume latency QoS (-1 to disable; default: 1000 µs)
-T / --timely               Enable adaptive time-slice feedback
-k                          Enable per-CPU kthread prioritization (experimental)
-d                          Enable BPF debug output via trace_pipe
-v                          Verbose output including libbpf details
--no-sticky-tasks           Disable sticky task dispatch
--no-local-pcpu             Disable per-CPU task local dispatch
--no-preferred-idle-scan    Disable capacity-sorted idle CPU selection
--no-cpufreq                Disable scheduler-driven frequency scaling
--no-group-iact             Disable per-process interactivity scoring
--no-warp                   Disable the interactive priority dispatch window
--no-ecore-consolidate      Disable E-core idle consolidation
--disable-smt               Disable SMT awareness
--disable-numa              Disable NUMA awareness
--stats <intv>              Live statistics at the given interval (seconds)
--monitor <intv>            Statistics monitoring only (no scheduler)
```

---

## 10. Overhead

The overhead relative to a minimal sched_ext skeleton is concentrated in `enqueue` and `select_cpu`. The `dispatch` path — the tightest loop under sustained load — has no structural change from upstream `scx_bpfland`.

| Function | Added cost | Notes |
| :--- | :--- | :--- |
| `select_cpu` | +1 ktime call | `enqueue_ts` sampled once; reused for TIMELY and warp |
| `enqueue` | +2–4 DSQ nr\_queued calls | For starvation enforcement; zero when tier DSQs are empty |
| `dispatch` | +4 DSQ peek calls | One per tier head task; WCEL check is 2 comparisons |
| `running` | +1 TIMELY sample | Only when `timely_enabled`; no-op otherwise |
| `stopping` | +1 map lookup | Group interactivity score update (hash map, ~5 ns) |
| `exit_task` | +1 map delete | Only on last thread exit of a process |

All per-task TIMELY state shares the `task_ctx` allocation with the core scheduling fields. No additional per-task allocations are introduced.

---

## 11. Vocabulary

### Scheduling

| Term | Definition |
| :--- | :--- |
| **Tier** | Priority level (T0–T2). Controls dispatch order, starvation budget, and vtime offset. |
| **Vtime** | Virtual runtime used as DSQ sort key. Includes tier offset so cross-tier comparison needs no branching. |
| **Lag** | Credit given to sleeping tasks: the more a task sleeps, the earlier its vtime deadline relative to CPU-bound peers. |
| **Starvation budget** | Maximum wall-clock time a tier's head task can wait before winning dispatch unconditionally. |
| **Priority window** | The 8 ms window during which T0 tasks preempt all EDF ordering. Tracked per-CPU. |
| **Interactivity score** | Per-process 0–16 value derived from `blocked / (blocked + cpu_used)`. High score → interactive. |
| **Decay generation** | Integer counter tracking how many 500 ms windows have elapsed; used to gate score decay to once per window. |
| **Gain** | Fixed-point TIMELY multiplier in [128..1024] applied to `slice_max` to produce the per-task slice. |
| **HAI streak** | Count of consecutive TIMELY control intervals where gain remained below 75% of max. |
| **EWMA** | Exponential Weighted Moving Average. Used for wakeup\_freq, queue delay, and gradient. |

### Hardware

| Term | Definition |
| :--- | :--- |
| **P-core** | Performance core — higher cpu\_capacity, higher power draw. |
| **E-core** | Efficiency core — lower cpu\_capacity, lower power draw, capable of deep C-states. |
| **C-state** | CPU idle power state. Deeper states save more power but have longer exit latencies. |
| **C6/CC6** | Deep idle state on Intel/AMD (~130–150 µs exit latency). Permitted by the 1000 µs latency QoS preset. |
| **C10/PC10** | Very deep package idle state (500 µs–2 ms exit latency). Blocked by the 1000 µs preset. |
| **LLC** | Last Level Cache. Cores sharing an LLC have lower inter-core communication latency. |
| **SMT** | Simultaneous Multi-Threading. Two logical CPUs per physical core; aura avoids placing a task on an SMT sibling whose physical core is already active when a fully-idle core is available. |
| **EPP** | Energy Performance Preference. Linux kernel attribute used to identify P-core and E-core rankings for primary domain selection. |

### Research Sources

| Feature | Derived from |
| :--- | :--- |
| Vruntime EDF with lag-based interactivity | scx\_bpfland |
| Three-tier classification with per-process scoring | XNU Clutch scheduler concepts (Apple open-source) |
| Delay-driven adaptive slice feedback | TIMELY research (SIGCOMM 2015 — Swift congestion control adapted for CPU scheduling) |
| Priority dispatch window | XNU root-bucket warp mechanism |
| Starvation enforcement budgets | XNU WCEL (Worst-Case Execution Latency) per-bucket guarantees |
| E-core idle consolidation | XNU AMP spill/consolidation (inverted for power saving) |
| Per-CPU SMT-aware idle selection | scx\_bpfland preferred idle scan |
