## **A.U.R.A**

**Adaptive · Utilisation · Responsive · Architecture**

> **ABSTRACT**: `scx_aura` is a BPF CPU scheduler built on [sched_ext](https://github.com/sched-ext/scx), designed for **laptop workloads** that demand both low-latency responsiveness and long battery life. It classifies every task by observed sleep and CPU behaviour, routes work through a 3-tier priority system, and actively manages CPU placement and frequency to keep efficiency cores idle.
>
> - **3-Tier Classification** Tasks sorted into Interactive / Default / Background by per-process sleep-to-CPU ratio and per-task wakeup frequency
> - **Bounded Starvation Avoidance** Each tier has a hard latency budget (0 ms / 75 ms / 250 ms); an expired budget wins dispatch unconditionally only if nothing higher is runnable — otherwise it's granted a single bounded quantum before the decision is re-evaluated, so a lower tier can never monopolize the CPU once a higher tier is also waiting
> - **Depleting Warp Budget** Each tier gets a per-tier budget (8 ms / 2 ms / 0 ms) to preempt EDF ordering ahead of lower tiers; the budget drains in real time while spent and only refills when the tier wins its next dispatch fairly, so sustained arrivals can never warp indefinitely
> - **Adaptive Time-Slice Feedback** Per-task queue delay drives a fixed-point gain that scales each task's slice up or down; gradient detection and a high-activity-index streak catch rising and sustained congestion
> - **E-Core Idle Consolidation with Passive Rebalance** When the system is below 75% load, all tiers avoid efficiency cores at enqueue time, and an idle performance core will reclaim work already sitting on an efficiency core's queue rather than let it sit there while the P-core idles
> - **Per-Process Interactivity Scoring** Each process group accumulates CPU-used and voluntary-sleep time; the ratio updates a 0–16 score that drives tier placement and decays over time so past behaviour does not permanently define a group

## Navigation

- [1. Quick Start](#1-quick-start)
- [2. Philosophy](#2-philosophy)
- [3. 3-Tier System](#3-3-tier-system)
- [4. Warp Budget and Starvation Avoidance](#4-warp-budget-and-starvation-avoidance)
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

## 4. Warp Budget and Starvation Avoidance

`scx_aura` lets a tier jump ahead of normal EDF ordering through two related but distinct mechanisms: a **depleting warp budget** (a tier spending its own allowance to preempt) and a **bounded starvation-avoidance window** (a tier being granted one quantum because it has aged past its latency budget). Both are modeled on the equivalent mechanisms in XNU's Clutch scheduler, and both are deliberately *bounded* — neither can let a tier monopolize the CPU indefinitely.

### Warp Budget

Each tier holds a per-tier budget that lets it jump ahead of lower tiers in the EDF race. The budget is shared across CPUs (not per-CPU), drains in real time while a tier is actively using it, and is only refilled when the tier wins its next dispatch *fairly* — on deadline alone, without spending warp. A tier with continuous arrivals cannot warp forever: once its budget is exhausted, it falls back to normal EDF until it earns a fair win and refills.

| Tier | Warp budget | Can warp over |
| :--- | :--- | :--- |
| Interactive | 8 ms | Default, Background |
| Default | 2 ms | Background only |
| Background | 0 ms | — (never warps) |

Warp is only attempted when there is actually something to jump ahead of — if the lower tiers are empty, normal EDF already dispatches the higher tier next, so no budget is spent. Warp also never overrides a task already placed directly on a CPU's own per-CPU dispatch queue (sticky tasks, kthreads, migration-pinned tasks) if that task's own deadline is earlier — those placements are never preempted by a tier's warp shortcut.

### Starvation Avoidance

Independent of warp, each tier has a hard latency budget enforced by tracking the wall-clock timestamp of the oldest task in each tier's queue.

| Tier | Latency budget |
| :--- | :--- |
| Interactive | 0 ms (always wins) |
| Default | 75 ms |
| Background | 250 ms |

When a tier's head task ages past its budget, what happens next depends on whether anything *higher* is currently runnable:

- **Nothing higher runnable:** the aged tier wins dispatch unconditionally — there is nothing to bound against, so this matches XNU's natural-order selection.
- **A higher tier is also runnable:** the aged tier is instead granted a single **1 ms starvation-avoidance window**. It wins dispatch for the duration of that window, then the decision is re-evaluated from scratch. If the tier is still starved on the next pass, a fresh window opens. This gives the aged tier roughly one quantum each time it reaches the front of the queue, rather than letting it lock out higher tiers for as long as its head stays old.

The starvation timestamp only clears when the tier's queue becomes empty, so the budget correctly tracks the age of the queue's oldest waiting task, not just the most recently dispatched one.

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

### Passive Rebalance

Consolidation only governs *new* placements — it does not by itself move work that already landed on an E-core before consolidation kicked in. To close that gap, when a performance core finds nothing of its own to dispatch while consolidation is active, it checks every efficiency core's per-CPU dispatch queue and claims any task waiting there before going idle.

This is deliberately passive: it never sends an IPI or kick to the efficiency core, and it never touches a task that is already running — it only reclaims queued (not yet running) work, using the same dispatch-queue move primitive the scheduler already uses for its own per-CPU queue. The effect is that a P-core finishing its own work absorbs spillover from an E-core instead of letting that E-core stay powered to finish it, while the P-core would otherwise have gone idle regardless.

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
enqueue    → sample enqueue_ts, tier DSQ insert, kick CPUs so dispatch runs promptly for T0
dispatch   → warp budget check → starvation check → EDF across all DSQs → E-core rebalance pull → keep_running
running    → sample delay, update TIMELY gain, advance vtime_now from raw vtime
stopping   → advance vruntime, update cpufreq utilisation, update group score
runnable   → update wakeup_freq EWMA, accumulate group blocked time
exit_task  → delete group score map entry when last thread exits
```

### Key Data Structures

| Structure | Purpose |
| :--- | :--- |
| `task_ctx` | Per-task: vruntime, wakeup\_freq, avg\_runtime, tier, TIMELY gain/delay/gradient/HAI |
| `cpu_ctx` | Per-CPU: runtime accumulators, frequency tracking, SMT sibling mask |
| `group_iact` | Per-tgid: cpu\_used\_ns, blocked\_accum\_ns, score, tier, decay generation counter |

Warp budgets and starvation-avoidance window state are tracked globally per tier (shared across CPUs), not per-CPU — both mechanisms refill or reset based on which tier wins dispatch, not on which CPU happens to be running it.

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
| Warp budget | On | — | Per-tier depleting preemption budget active (`--no-warp` to disable) |
| E-core consolidation + rebalance | On | — | Active below 75% system load; idle P-cores reclaim E-core spillover |
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
--no-warp                   Disable per-tier warp budget (EDF-only ordering, no preemption)
--no-ecore-consolidate      Disable E-core idle consolidation
--disable-smt               Disable SMT awareness
--disable-numa              Disable NUMA awareness
--stats <intv>              Live statistics at the given interval (seconds)
--monitor <intv>            Statistics monitoring only (no scheduler)
```

---

## 10. Overhead

The overhead relative to a minimal sched_ext skeleton is concentrated in `enqueue`, `select_cpu`, and `dispatch`. Unlike earlier revisions, `dispatch` is no longer a thin wrapper around upstream `scx_bpfland`'s loop — the warp-budget accounting, the bounded starvation window, and the E-core rebalance pull all run there. Compiled with `clang -O2 -target bpf`, `dispatch` is the largest callback in the scheduler at roughly 1000 BPF instructions, still well within normal range for this class of program and with no observed verifier-complexity warnings under `-Wall -Wextra`.

| Function | Added cost | Notes |
| :--- | :--- | :--- |
| `select_cpu` | +1 ktime call | `enqueue_ts` sampled once; reused for TIMELY and warp |
| `enqueue` | +2–4 DSQ nr\_queued calls | For starvation enforcement; zero when tier DSQs are empty |
| `dispatch` | +4 DSQ peek calls, up to 2 deadline-min comparisons per warp attempt, a bounded CAS retry (≤16 iterations) when warp budget is actually spent, and — only when a P-core is otherwise about to idle during consolidation — a bounded scan of efficiency-core queues | Warp and starvation checks are skipped entirely when the relevant tier DSQs are empty; the rebalance scan only runs on the idle-fallback path, not on every dispatch call |
| `running` | +1 TIMELY sample | Only when `timely_enabled`; no-op otherwise |
| `stopping` | +1 map lookup | Group interactivity score update (hash map, ~5 ns) |
| `exit_task` | +1 map delete | Only on last thread exit of a process |

All per-task TIMELY state shares the `task_ctx` allocation with the core scheduling fields. No additional per-task allocations are introduced. Warp and starvation-avoidance state are small fixed-size global arrays (one entry per tier), not per-task or per-CPU allocations.

---

## 11. Vocabulary

### Scheduling

| Term | Definition |
| :--- | :--- |
| **Tier** | Priority level (T0–T2). Controls dispatch order, starvation budget, and vtime offset. |
| **Vtime** | Virtual runtime used as DSQ sort key. Includes tier offset so cross-tier comparison needs no branching. |
| **Lag** | Credit given to sleeping tasks: the more a task sleeps, the earlier its vtime deadline relative to CPU-bound peers. |
| **Starvation budget** | Maximum wall-clock time a tier's head task can wait before triggering starvation avoidance — unconditional if nothing higher is runnable, otherwise a bounded window. |
| **Warp budget** | Per-tier allowance (shared across CPUs) that lets a tier jump ahead of lower tiers in EDF. Drains while spent; refills only on a fair (non-warp, non-starvation-override) win. |
| **Starvation-avoidance window** | The single bounded quantum (1 ms) a tier is granted when its latency budget expires while a higher tier is also runnable. Re-opens if the tier is still starved afterward; tracked per tier, not per-CPU. |
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
| Depleting per-tier warp budget | XNU root-bucket warp mechanism (`scrb_warp_remaining`), including its fairness-gated refill |
| Bounded starvation-avoidance window | XNU WCEL (Worst-Case Execution Latency) per-bucket guarantees, including the one-quantum starvation-avoidance window in `sched_clutch_root_highest_root_bucket()` |
| E-core idle consolidation | XNU AMP spill/consolidation (inverted for power saving) |
| Passive E-core → P-core rebalance | XNU `sched_amp_balance()` |
| Per-CPU SMT-aware idle selection | scx\_bpfland preferred idle scan |
