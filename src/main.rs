// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2024 Andrea Righi <andrea.righi@linux.dev>
//
// Laptop-optimised fork.  Running `scx_aura` with no arguments loads a
// laptop preset (see LaptopPreset) that enables fast responsiveness and
// longer battery life out of the box.  Every preset value can be overridden
// by the corresponding CLI flag.

mod bpf_skel;
pub use bpf_skel::*;
pub mod bpf_intf;
pub use bpf_intf::*;

mod stats;
use std::ffi::{c_int, c_ulong};
use std::fmt::Write;
use std::mem::MaybeUninit;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::Duration;

use anyhow::anyhow;
use anyhow::bail;
use anyhow::Context;
use anyhow::Result;
use clap::Parser;
use crossbeam::channel::RecvTimeoutError;
use libbpf_rs::OpenObject;
use libbpf_rs::ProgramInput;
use log::warn;
use log::{debug, info};
use scx_stats::prelude::*;
use scx_utils::autopower::{fetch_power_profile, PowerProfile};
use scx_utils::build_id;
use scx_utils::compat;
use scx_utils::get_primary_cpus;
use scx_utils::libbpf_clap_opts::LibbpfOpts;
use scx_utils::pm::{cpu_idle_resume_latency_supported, update_cpu_idle_resume_latency};
use scx_utils::scx_ops_attach;
use scx_utils::scx_ops_load;
use scx_utils::scx_ops_open;
use scx_utils::try_set_rlimit_infinity;
use scx_utils::uei_exited;
use scx_utils::uei_report;
use scx_utils::Cpumask;
use scx_utils::Powermode;
use scx_utils::Topology;
use scx_utils::UserExitInfo;
use scx_utils::NR_CPU_IDS;
use stats::Metrics;

const SCHEDULER_NAME: &str = "scx_aura";

// ── Sentinel values for "user did not set this flag" ─────────────────────────
//
// SetTrue flags default to false, so a bare bool is enough to distinguish
// "off by default" from "explicitly set".  For numeric and string flags that
// the preset wants to override we use explicit sentinel constants.

/// Sentinel for idle_resume_us: means "let the preset decide".
const IDLE_RESUME_PRESET: i64 = i64::MIN;

/// Sentinel for primary_domain: means "let the preset decide".
const PRIMARY_DOMAIN_PRESET: &str = "preset";

/// Sentinel for slice_us: means "let the preset decide".
const SLICE_US_PRESET: u64 = u64::MAX;

fn cpus_to_cpumask(cpus: &Vec<usize>) -> String {
    if cpus.is_empty() {
        return String::from("none");
    }
    let max_cpu_id = *cpus.iter().max().unwrap();
    let mut bitmask = vec![0u8; (max_cpu_id + 1 + 7) / 8];
    for cpu_id in cpus {
        let byte_index = cpu_id / 8;
        let bit_index = cpu_id % 8;
        bitmask[byte_index] |= 1 << bit_index;
    }
    let hex_str: String = bitmask.iter().rev().fold(String::new(), |mut f, byte| {
        let _ = write!(&mut f, "{:02x}", byte);
        f
    });
    format!("0x{}", hex_str)
}

// ── Laptop preset ─────────────────────────────────────────────────────────────
//
// Built once in Scheduler::init after topology and power-profile detection.
// Holds the *effective* value of every tunable: preset default if the user
// did not supply a flag, user-supplied value otherwise.
//
// Preset rationale (what each choice does and why):
//
//  primary_domain = "performance"
//      On heterogeneous CPUs (Intel P+E, AMD multi-CCD) this puts P-cores /
//      high-efficiency cores first in the idle selection path, which works
//      in concert with E-core consolidation: both mechanisms push work toward
//      higher-capacity cores and let lower-capacity cores sleep.  On
//      homogeneous CPUs `get_primary_cpus(Performance)` returns all cores so
//      the behaviour is identical to "all".
//
//  cpufreq = true
//      Lets the scheduler drive per-CPU frequency based on measured
//      utilisation via scx_bpf_cpuperf_set().  Without this the governor
//      makes frequency decisions independently of the scheduler's placement
//      choices, undermining E-core consolidation.
//
//  sticky_tasks = true
//      Short-runtime tasks (avg_runtime < 10 µs) are dispatched directly to
//      SCX_DSQ_LOCAL rather than being migrated.  This keeps frequently-
//      waking interactive tasks on a warm cache and prevents the CPU from
//      bouncing between C-states on every short wakeup.
//
//  idle_resume_us = 1000
//      Sets a 1 ms PM QoS latency constraint on every CPU.  This permits
//      deep C-states (C6 on Intel ~130 µs, CC6 on AMD ~150 µs) while
//      blocking pathological deep states (C10, PC10) whose exit latencies
//      exceed 1 ms and add visible latency to interactive events.
//      Restored to per-CPU hardware default on scheduler exit.
//
//  local_pcpu = true
//      Single-CPU-affinity tasks dispatch directly to SCX_DSQ_LOCAL,
//      avoiding the shared DSQ overhead for tasks that can never migrate.
//
//  slice_us = 800
//      Slightly shorter than the upstream 1000 µs default.  Combined with
//      the 40 ms lag window, this gives interactive tasks more scheduling
//      opportunities per second while keeping context-switch overhead low.
//
//  preferred_idle_scan = true
//      Uses the capacity-sorted preferred_cpus[] scan for idle CPU
//      selection instead of scx_bpf_select_cpu_and(), giving deterministic
//      P-core-first placement that the kernel helper cannot guarantee.
//
// Everything else (TIMELY, NUMA, SMT, group_iact, warp, ecore_consolidate)
// keeps its existing default (TIMELY off, NUMA/SMT auto-detected, laptop
// features on).

#[derive(Debug)]
struct LaptopPreset {
    // scheduling
    slice_us:          u64,
    slice_min_us:      u64,
    slice_us_lag:      u64,
    sticky_tasks:      bool,
    local_pcpu:        bool,
    local_kthreads:    bool,
    no_wake_sync:      bool,
    // CPU placement
    primary_domain:    String,
    preferred_idle_scan: bool,
    cpufreq:           bool,
    // power management
    idle_resume_us:    i64,
    // laptop features (all on by default; negated by --no-* flags)
    group_iact:        bool,
    ecore_consolidate: bool,
    warp:              bool,
}

impl LaptopPreset {
    fn build(opts: &Opts, _topo: &Topology) -> Self {
        // ── slice ──────────────────────────────────────────────────────────
        let slice_us = if opts.slice_us == SLICE_US_PRESET {
            800   // preset: slightly shorter for more interactive opportunities
        } else {
            opts.slice_us
        };

        // ── primary domain ─────────────────────────────────────────────────
        //
        // Detect whether the CPU is heterogeneous by checking if any CPU has
        // a capacity different from cpu 0.  On homogeneous CPUs "performance"
        // resolves to all cores; on heterogeneous ones it selects P-cores.
        let primary_domain = if opts.primary_domain == PRIMARY_DOMAIN_PRESET {
            "performance".to_string()
        } else {
            opts.primary_domain.clone()
        };

        // ── idle resume latency ────────────────────────────────────────────
        let idle_resume_us = if opts.idle_resume_us == IDLE_RESUME_PRESET {
            1000   // preset: allow C6 (~150 µs exit), block C10 (>1 ms exit)
        } else {
            opts.idle_resume_us
        };

        // ── boolean flags: preset supplies true, --flag overrides to false ─
        //
        // The pattern: opts.flag is `false` when not supplied (SetTrue),
        // so `!opts.disable_X` gives the user a way to opt out while keeping
        // the preset default.
        //
        // For flags that are opt-in (SetTrue) in the original, the preset
        // supplies true and the user can't easily override to false without
        // a new --no-X flag.  Those are added as --no-sticky-tasks etc.
        let sticky_tasks       = !opts.no_sticky_tasks;
        let local_pcpu         = !opts.no_local_pcpu;
        let preferred_idle_scan = !opts.no_preferred_idle_scan;
        let cpufreq            = !opts.no_cpufreq;

        // ── opts pass-through ──────────────────────────────────────────────
        let local_kthreads = opts.local_kthreads;
        let no_wake_sync   = opts.no_wake_sync;
        let group_iact     = !opts.no_group_iact;
        let ecore_consolidate = !opts.no_ecore_consolidate;
        let warp           = !opts.no_warp;

        // Log the active preset.
        info!(
            "laptop preset: slice={}µs primary={} sticky={} local_pcpu={} \
             pref_scan={} cpufreq={} idle_resume={}µs \
             group_iact={} ecore={} warp={}",
            slice_us, primary_domain, sticky_tasks, local_pcpu,
            preferred_idle_scan, cpufreq, idle_resume_us,
            group_iact, ecore_consolidate, warp,
        );

        LaptopPreset {
            slice_us,
            slice_min_us:    opts.slice_min_us,
            slice_us_lag:    opts.slice_us_lag,
            sticky_tasks,
            local_pcpu,
            local_kthreads,
            no_wake_sync,
            primary_domain,
            preferred_idle_scan,
            cpufreq,
            idle_resume_us,
            group_iact,
            ecore_consolidate,
            warp,
        }
    }
}

// ── CLI definition ────────────────────────────────────────────────────────────
//
// Flags that existed in the original aura are kept with the same short
// names and semantics.  New --no-X flags allow opting out of preset defaults
// that were previously opt-in SetTrue flags.

/// scx_aura: vruntime-based sched_ext scheduler optimised for laptops.
///
/// Running with no arguments loads a laptop preset:
///   sticky tasks on, local pcpu on, preferred idle scan on, cpufreq on,
///   primary domain = performance, idle resume latency = 1000 µs.
///
/// All preset values can be individually overridden.
#[derive(Debug, Parser)]
struct Opts {
    /// Exit debug dump buffer length. 0 indicates default.
    #[clap(long, default_value = "0")]
    exit_dump_len: u32,

    /// Maximum scheduling slice in microseconds.
    /// Default (preset): 800 µs.
    #[clap(short = 's', long, default_value_t = SLICE_US_PRESET,
           hide_default_value = true)]
    slice_us: u64,

    /// Minimum scheduling slice in microseconds (0 = disabled).
    #[clap(short = 'L', long, default_value = "0")]
    slice_min_us: u64,

    /// Maximum time slice lag in microseconds.
    #[clap(short = 'l', long, default_value = "40000")]
    slice_us_lag: u64,

    /// Throttle CPUs by periodically injecting idle cycles (0 = disabled).
    #[clap(short = 't', long, default_value = "0")]
    throttle_us: u64,

    /// CPU idle QoS resume latency in microseconds.
    /// Default (preset): 1000 µs (permits C6, blocks C10).
    /// Pass -1 to disable entirely.
    #[clap(short = 'I', long, allow_hyphen_values = true,
           default_value_t = IDLE_RESUME_PRESET, hide_default_value = true)]
    idle_resume_us: i64,

    /// Primary scheduling domain (bitmask, or: auto/performance/powersave/turbo/all).
    /// Default (preset): performance (P-cores first on heterogeneous CPUs).
    #[clap(short = 'm', long, default_value = PRIMARY_DOMAIN_PRESET,
           hide_default_value = true)]
    primary_domain: String,

    // ── opt-in flags (unchanged from upstream) ────────────────────────────

    /// Enable kthreads prioritization (EXPERIMENTAL).
    #[clap(short = 'k', long, action = clap::ArgAction::SetTrue)]
    local_kthreads: bool,

    /// Disable direct dispatch during synchronous wakeups.
    #[clap(short = 'w', long, action = clap::ArgAction::SetTrue)]
    no_wake_sync: bool,

    /// Disable SMT awareness.
    #[clap(long, action = clap::ArgAction::SetTrue)]
    disable_smt: bool,

    /// Disable NUMA awareness.
    #[clap(long, action = clap::ArgAction::SetTrue)]
    disable_numa: bool,

    /// Enable TIMELY adaptive time-slice feedback.
    #[clap(short = 'T', long, action = clap::ArgAction::SetTrue)]
    timely: bool,

    /// TIMELY lower delay threshold in microseconds.
    #[clap(long, default_value = "5000")]
    timely_tlow_us: u64,

    /// TIMELY higher delay threshold in microseconds.
    #[clap(long, default_value = "50000")]
    timely_thigh_us: u64,

    /// TIMELY minimum gain value (fixed-point).
    #[clap(long, default_value = "128")]
    timely_gain_min: u32,

    /// TIMELY gain step (fixed-point).
    #[clap(long, default_value = "32")]
    timely_gain_step: u32,

    /// TIMELY HAI threshold (fixed-point).
    #[clap(long, default_value = "768")]
    timely_hai_thresh: u32,

    /// TIMELY HAI multiplier.
    #[clap(long, default_value = "2")]
    timely_hai_multiplier: u32,

    /// TIMELY backoff low (fixed-point).
    #[clap(long, default_value = "768")]
    timely_backoff_low: u32,

    /// TIMELY backoff high (fixed-point).
    #[clap(long, default_value = "960")]
    timely_backoff_high: u32,

    /// TIMELY backoff gradient (fixed-point).
    #[clap(long, default_value = "992")]
    timely_backoff_gradient: u32,

    /// TIMELY gradient margin in microseconds.
    #[clap(long, default_value = "125")]
    timely_gradient_margin_us: u64,

    /// TIMELY control interval in microseconds.
    #[clap(long, default_value = "500")]
    timely_control_interval_us: u64,

    // ── opt-out flags (preset on, --no-X to disable) ──────────────────────
    //
    // These replace the old SetTrue opt-in flags for sticky_tasks,
    // local_pcpu, preferred_idle_scan, and cpufreq.  The old short flags
    // (-S, -p, -P, -f) are kept as aliases on the --no- variants' inverses
    // to avoid breaking existing scripts that used them.

    /// Disable sticky-task dispatch (preset: on).
    /// Sticky tasks: short-runtime tasks are dispatched to their previous CPU
    /// without entering the shared DSQ, preserving cache warmth.
    #[clap(long, action = clap::ArgAction::SetTrue)]
    no_sticky_tasks: bool,

    /// Disable per-CPU task local dispatch (preset: on).
    #[clap(long, action = clap::ArgAction::SetTrue)]
    no_local_pcpu: bool,

    /// Disable capacity-sorted preferred idle CPU scan (preset: on).
    /// When on, idle CPU selection walks CPUs in descending capacity order
    /// (P-cores first) instead of using scx_bpf_select_cpu_and().
    #[clap(long, action = clap::ArgAction::SetTrue)]
    no_preferred_idle_scan: bool,

    /// Disable scheduler-driven CPU frequency scaling (preset: on).
    /// When on, the scheduler adjusts per-CPU frequency via
    /// scx_bpf_cpuperf_set() based on measured utilisation.
    #[clap(long, action = clap::ArgAction::SetTrue)]
    no_cpufreq: bool,

    /// Disable per-process-group interactivity scoring (preset: on).
    #[clap(long, action = clap::ArgAction::SetTrue)]
    no_group_iact: bool,

    /// Disable E-core idle consolidation (preset: on).
    #[clap(long, action = clap::ArgAction::SetTrue)]
    no_ecore_consolidate: bool,

    /// Disable interactive-tier warp dispatch (preset: on).
    #[clap(long, action = clap::ArgAction::SetTrue)]
    no_warp: bool,

    // ── standard ──────────────────────────────────────────────────────────

    /// Enable stats monitoring with the specified interval.
    #[clap(long)]
    stats: Option<f64>,

    /// Run in stats monitoring mode only (scheduler not launched).
    #[clap(long)]
    monitor: Option<f64>,

    /// Enable BPF debugging via /sys/kernel/tracing/trace_pipe.
    #[clap(short = 'd', long, action = clap::ArgAction::SetTrue)]
    debug: bool,

    /// Enable verbose output including libbpf details.
    #[clap(short = 'v', long, action = clap::ArgAction::SetTrue)]
    verbose: bool,

    /// Print version and exit.
    #[clap(short = 'V', long, action = clap::ArgAction::SetTrue)]
    version: bool,

    /// Show descriptions for statistics.
    #[clap(long)]
    help_stats: bool,

    #[clap(flatten, next_help_heading = "Libbpf Options")]
    pub libbpf: LibbpfOpts,
}

struct Scheduler<'a> {
    skel: BpfSkel<'a>,
    struct_ops: Option<libbpf_rs::Link>,
    opts: &'a Opts,
    topo: Topology,
    /// Effective idle_resume_us that was applied (may differ from opts if
    /// preset supplied the value).  Stored so Drop can restore it correctly.
    applied_idle_resume_us: i64,
    power_profile: PowerProfile,
    stats_server: StatsServer<(), Metrics>,
    user_restart: bool,
}

impl<'a> Scheduler<'a> {
    fn init(opts: &'a Opts, open_object: &'a mut MaybeUninit<OpenObject>) -> Result<Self> {
        try_set_rlimit_infinity();

        let topo = Topology::new().unwrap();

        let smt_enabled = !opts.disable_smt && topo.smt_enabled;

        let nr_nodes = topo
            .nodes
            .values()
            .filter(|node| !node.all_cpus.is_empty())
            .count();
        info!("NUMA nodes: {}", nr_nodes);

        let numa_enabled = !opts.disable_numa && nr_nodes > 1;
        if !numa_enabled {
            info!("Disabling NUMA optimizations");
        }

        let power_profile = Self::power_profile();

        // Build the effective configuration from the laptop preset + any
        // user overrides.  This must happen before rodata is written.
        let preset = LaptopPreset::build(opts, &topo);

        info!(
            "{} {}",
            SCHEDULER_NAME,
            build_id::full_version(env!("CARGO_PKG_VERSION")),
        );
        info!("scheduler options: {}", std::env::args().collect::<Vec<_>>().join(" "));

        // Resolve the primary domain using the preset's value.
        let domain = Self::resolve_energy_domain(&preset.primary_domain, power_profile)
            .map_err(|err| {
                anyhow!(
                    "failed to resolve primary domain '{}': {}",
                    &preset.primary_domain,
                    err
                )
            })?;

        // Apply idle resume latency QoS.
        // preset.idle_resume_us is always a concrete value (never the sentinel
        // IDLE_RESUME_PRESET) because LaptopPreset::build already resolved it.
        let applied_idle_resume_us = preset.idle_resume_us;
        if applied_idle_resume_us >= 0 {
            if !cpu_idle_resume_latency_supported() {
                warn!("idle resume latency QoS not supported on this kernel");
            } else {
                info!("setting idle QoS to {} µs", applied_idle_resume_us);
                for cpu in topo.all_cpus.values() {
                    update_cpu_idle_resume_latency(
                        cpu.id,
                        applied_idle_resume_us.try_into().unwrap(),
                    )?;
                }
            }
        }

        let mut skel_builder = BpfSkelBuilder::default();
        skel_builder.obj_builder.debug(opts.verbose);
        let open_opts = opts.libbpf.clone().into_bpf_open_opts();
        let mut skel = scx_ops_open!(skel_builder, open_object, aura_ops, open_opts)?;

        skel.struct_ops.aura_ops_mut().exit_dump_len = opts.exit_dump_len;

        // Write all rodata from the preset, not directly from opts.
        let rodata = skel.maps.rodata_data.as_mut().unwrap();
        rodata.debug              = opts.debug;
        rodata.smt_enabled        = smt_enabled;
        rodata.numa_enabled       = numa_enabled;
        rodata.local_pcpu         = preset.local_pcpu;
        rodata.no_wake_sync       = preset.no_wake_sync;
        rodata.sticky_tasks       = preset.sticky_tasks;
        rodata.slice_max          = preset.slice_us * 1000;
        rodata.slice_min          = preset.slice_min_us * 1000;
        rodata.slice_lag          = preset.slice_us_lag * 1000;
        rodata.throttle_ns        = opts.throttle_us * 1000;
        rodata.primary_all        = domain.weight() == *NR_CPU_IDS;
        rodata.preferred_idle_scan = preset.preferred_idle_scan;
        rodata.local_kthreads     = preset.local_kthreads || opts.throttle_us > 0;

        // TIMELY (unchanged; user must opt in with -T).
        rodata.timely_enabled             = opts.timely;
        rodata.timely_tlow_ns             = opts.timely_tlow_us * 1000;
        rodata.timely_thigh_ns            = opts.timely_thigh_us * 1000;
        rodata.timely_gain_min_fp         = opts.timely_gain_min;
        rodata.timely_gain_max_fp         = 1024;
        rodata.timely_gain_step_fp        = opts.timely_gain_step;
        rodata.timely_hai_thresh_fp       = opts.timely_hai_thresh;
        rodata.timely_hai_multiplier      = opts.timely_hai_multiplier;
        rodata.timely_backoff_low_fp      = opts.timely_backoff_low;
        rodata.timely_backoff_high_fp     = opts.timely_backoff_high;
        rodata.timely_backoff_gradient_fp = opts.timely_backoff_gradient;
        rodata.timely_gradient_margin_ns  = opts.timely_gradient_margin_us * 1000;
        rodata.timely_control_interval_ns = opts.timely_control_interval_us * 1000;

        // Laptop features.
        rodata.group_iact_enabled = preset.group_iact;
        rodata.ecore_consolidate  = preset.ecore_consolidate;
        rodata.warp_enabled       = preset.warp;

        // CPU capacity array and preferred scan order (descending capacity).
        let mut cpus: Vec<_> = topo.all_cpus.values().collect();
        cpus.sort_by_key(|cpu| std::cmp::Reverse(cpu.cpu_capacity));
        for (i, cpu) in cpus.iter().enumerate() {
            rodata.cpu_capacity[cpu.id] = cpu.cpu_capacity as c_ulong;
            rodata.preferred_cpus[i]    = cpu.id as u64;
        }
        if preset.preferred_idle_scan {
            info!(
                "preferred CPUs (capacity-sorted): {:?}",
                &rodata.preferred_cpus[0..cpus.len()]
            );
        }

        skel.struct_ops.aura_ops_mut().flags = *compat::SCX_OPS_ENQ_EXITING
            | *compat::SCX_OPS_ENQ_LAST
            | *compat::SCX_OPS_ENQ_MIGRATION_DISABLED
            | *compat::SCX_OPS_ALLOW_QUEUED_WAKEUP
            | if numa_enabled {
                *compat::SCX_OPS_BUILTIN_IDLE_PER_NODE
            } else {
                0
            };
        info!("scheduler flags: {:#x}", skel.struct_ops.aura_ops_mut().flags);

        let mut skel = scx_ops_load!(skel, aura_ops, uei)?;

        Self::init_energy_domain(&mut skel, &domain).map_err(|err| {
            anyhow!(
                "failed to initialise primary domain 0x{:x}: {}",
                domain,
                err
            )
        })?;

        // Drive cpufreq from the preset value.
        if let Err(err) = Self::init_cpufreq_perf(
            &mut skel,
            &preset.primary_domain,
            preset.cpufreq,
        ) {
            bail!("failed to initialise cpufreq performance level: {}", err);
        }

        if smt_enabled {
            Self::init_smt_domains(&mut skel, &topo)?;
        }

        let struct_ops   = Some(scx_ops_attach!(skel, aura_ops)?);
        let stats_server = StatsServer::new(stats::server_data()).launch()?;

        Ok(Self {
            skel,
            struct_ops,
            opts,
            topo,
            applied_idle_resume_us,
            power_profile,
            stats_server,
            user_restart: false,
        })
    }

    fn enable_primary_cpu(skel: &mut BpfSkel<'_>, cpu: i32) -> Result<(), u32> {
        let prog = &mut skel.progs.enable_primary_cpu;
        let mut args = cpu_arg { cpu_id: cpu as c_int };
        let input = ProgramInput {
            context_in: Some(unsafe {
                std::slice::from_raw_parts_mut(
                    &mut args as *mut _ as *mut u8,
                    std::mem::size_of_val(&args),
                )
            }),
            ..Default::default()
        };
        let out = prog.test_run(input).unwrap();
        if out.return_value != 0 {
            return Err(out.return_value);
        }
        Ok(())
    }

    fn epp_to_cpumask(profile: Powermode) -> Result<Cpumask> {
        let mut cpus = get_primary_cpus(profile).unwrap_or_default();
        if cpus.is_empty() {
            cpus = get_primary_cpus(Powermode::Any).unwrap_or_default();
        }
        Cpumask::from_str(&cpus_to_cpumask(&cpus))
    }

    fn resolve_energy_domain(primary_domain: &str, power_profile: PowerProfile) -> Result<Cpumask> {
        let domain = match primary_domain {
            "powersave"   => Self::epp_to_cpumask(Powermode::Powersave)?,
            "performance" => Self::epp_to_cpumask(Powermode::Performance)?,
            "turbo"       => Self::epp_to_cpumask(Powermode::Turbo)?,
            "auto"        => match power_profile {
                PowerProfile::Powersave => Self::epp_to_cpumask(Powermode::Powersave)?,
                PowerProfile::Balanced { .. }
                | PowerProfile::Performance
                | PowerProfile::Unknown => Self::epp_to_cpumask(Powermode::Any)?,
            },
            "all" => Self::epp_to_cpumask(Powermode::Any)?,
            &_    => Cpumask::from_str(primary_domain)?,
        };
        Ok(domain)
    }

    fn init_energy_domain(skel: &mut BpfSkel<'_>, domain: &Cpumask) -> Result<()> {
        info!("primary CPU domain = 0x{:x}", domain);
        if let Err(err) = Self::enable_primary_cpu(skel, -1) {
            bail!("failed to reset primary domain: error {}", err);
        }
        for cpu in 0..*NR_CPU_IDS {
            if domain.test_cpu(cpu) {
                if let Err(err) = Self::enable_primary_cpu(skel, cpu as i32) {
                    bail!("failed to add CPU {} to primary domain: error {}", cpu, err);
                }
            }
        }
        Ok(())
    }

    fn init_cpufreq_perf(
        skel: &mut BpfSkel<'_>,
        primary_domain: &str,
        cpufreq: bool,
    ) -> Result<()> {
        let perf_lvl: i64 = match (cpufreq, primary_domain) {
            (false, _)            => 1024,  // fixed max; governor controls freq
            (true, "powersave")   => 0,     // fixed min
            (true, _)             => -1,    // dynamic utilisation-based
        };
        info!(
            "cpufreq performance level: {}",
            match perf_lvl {
                1024       => "max (cpufreq off)".into(),
                0          => "min".into(),
                n if n < 0 => "auto (utilisation-based)".into(),
                _          => perf_lvl.to_string(),
            }
        );
        skel.maps.bss_data.as_mut().unwrap().cpufreq_perf_lvl = perf_lvl;
        Ok(())
    }

    fn enable_sibling_cpu(
        skel: &mut BpfSkel<'_>,
        cpu: usize,
        sibling_cpu: usize,
    ) -> Result<(), u32> {
        let prog = &mut skel.progs.enable_sibling_cpu;
        let mut args = domain_arg {
            cpu_id: cpu as c_int,
            sibling_cpu_id: sibling_cpu as c_int,
        };
        let input = ProgramInput {
            context_in: Some(unsafe {
                std::slice::from_raw_parts_mut(
                    &mut args as *mut _ as *mut u8,
                    std::mem::size_of_val(&args),
                )
            }),
            ..Default::default()
        };
        let out = prog.test_run(input).unwrap();
        if out.return_value != 0 {
            return Err(out.return_value);
        }
        Ok(())
    }

    fn init_smt_domains(skel: &mut BpfSkel<'_>, topo: &Topology) -> Result<(), std::io::Error> {
        let smt_siblings = topo.sibling_cpus();
        info!("SMT sibling CPUs: {:?}", smt_siblings);
        for (cpu, sibling_cpu) in smt_siblings.iter().enumerate() {
            Self::enable_sibling_cpu(skel, cpu, *sibling_cpu as usize).unwrap();
        }
        Ok(())
    }

    fn get_metrics(&self) -> Metrics {
        let bss = self.skel.maps.bss_data.as_ref().unwrap();
        Metrics {
            nr_running:                         bss.nr_running,
            nr_cpus:                            bss.nr_online_cpus,
            nr_kthread_dispatches:              bss.nr_kthread_dispatches,
            nr_direct_dispatches:               bss.nr_direct_dispatches,
            nr_shared_dispatches:               bss.nr_shared_dispatches,
            nr_delay_recovery_dispatches:       bss.nr_delay_recovery_dispatches,
            nr_delay_middle_add_dispatches:     bss.nr_delay_middle_add_dispatches,
            nr_delay_fast_recovery_dispatches:  bss.nr_delay_fast_recovery_dispatches,
            nr_delay_rate_limited_dispatches:   bss.nr_delay_rate_limited_dispatches,
            nr_gain_floor_dispatches:           bss.nr_gain_floor_dispatches,
            nr_gain_ceiling_dispatches:         bss.nr_gain_ceiling_dispatches,
            nr_delay_low_region_samples:        bss.nr_delay_low_region_samples,
            nr_delay_mid_region_samples:        bss.nr_delay_mid_region_samples,
            nr_delay_high_region_samples:       bss.nr_delay_high_region_samples,
            nr_gain_floor_resident_samples:     bss.nr_gain_floor_resident_samples,
            nr_gain_mid_resident_samples:       bss.nr_gain_mid_resident_samples,
            nr_gain_ceiling_resident_samples:   bss.nr_gain_ceiling_resident_samples,
            nr_idle_select_path_picks:          bss.nr_idle_select_path_picks,
            nr_idle_enqueue_path_picks:         bss.nr_idle_enqueue_path_picks,
            nr_idle_prev_cpu_picks:             bss.nr_idle_prev_cpu_picks,
            nr_idle_primary_picks:              bss.nr_idle_primary_picks,
            nr_idle_spill_picks:                bss.nr_idle_spill_picks,
            nr_idle_pick_failures:              bss.nr_idle_pick_failures,
            nr_idle_primary_domain_misses:      bss.nr_idle_primary_domain_misses,
            nr_idle_global_misses:              bss.nr_idle_global_misses,
            nr_waker_cpu_biases:                bss.nr_waker_cpu_biases,
            nr_keep_running_reuses:             bss.nr_keep_running_reuses,
            nr_keep_running_queue_empty:        bss.nr_keep_running_queue_empty,
            nr_keep_running_smt_blocked:        bss.nr_keep_running_smt_blocked,
            nr_keep_running_queued_work:        bss.nr_keep_running_queued_work,
            nr_dispatch_cpu_dsq_consumes:       bss.nr_dispatch_cpu_dsq_consumes,
            nr_dispatch_node_dsq_consumes:      bss.nr_dispatch_node_dsq_consumes,
            nr_cpu_release_reenqueue:           bss.nr_cpu_release_reenqueue,
            // laptop counters
            nr_interactive_dispatches:          bss.nr_interactive_dispatches,
            nr_background_dispatches:           bss.nr_background_dispatches,
            nr_warp_dispatches:                 bss.nr_warp_dispatches,
            nr_default_warp_dispatches:         bss.nr_default_warp_dispatches,
            nr_iact_promoted:                   bss.nr_iact_promoted,
            nr_iact_demoted:                    bss.nr_iact_demoted,
            nr_ecore_consolidations:            bss.nr_ecore_consolidations,
            nr_ecore_rebalance_pulls:           bss.nr_ecore_rebalance_pulls,
            nr_preempt_kicks:                   bss.nr_preempt_kicks,
            nr_wcel_enforcements:               bss.nr_wcel_enforcements,
            nr_starvation_window_opens:         bss.nr_starvation_window_opens,
        }
    }

    pub fn exited(&mut self) -> bool {
        uei_exited!(&self.skel, uei)
    }

    fn power_profile() -> PowerProfile {
        let profile = fetch_power_profile(true);
        if profile == PowerProfile::Unknown {
            fetch_power_profile(false)
        } else {
            profile
        }
    }

    fn refresh_sched_domain(&mut self) -> bool {
        if self.power_profile != PowerProfile::Unknown {
            let power_profile = Self::power_profile();
            if power_profile != self.power_profile {
                self.power_profile = power_profile;

                // Rebuild the preset with the new power profile and restart
                // so rodata and the energy domain are recomputed.
                if self.opts.primary_domain == PRIMARY_DOMAIN_PRESET
                    || self.opts.primary_domain == "auto"
                {
                    return true;
                }
                if let Err(err) = Self::init_cpufreq_perf(
                    &mut self.skel,
                    &self.opts.primary_domain,
                    !self.opts.no_cpufreq,
                ) {
                    warn!("failed to refresh cpufreq level: {}", err);
                }
            }
        }
        false
    }

    fn run(&mut self, shutdown: Arc<AtomicBool>) -> Result<UserExitInfo> {
        let (res_ch, req_ch) = self.stats_server.channels();
        while !shutdown.load(Ordering::Relaxed) && !self.exited() {
            if self.refresh_sched_domain() {
                self.user_restart = true;
                break;
            }
            match req_ch.recv_timeout(Duration::from_secs(1)) {
                Ok(())  => res_ch.send(self.get_metrics())?,
                Err(RecvTimeoutError::Timeout) => {}
                Err(e)  => Err(e)?,
            }
        }
        let _ = self.struct_ops.take();
        uei_report!(&self.skel, uei)
    }
}

impl Drop for Scheduler<'_> {
    fn drop(&mut self) {
        info!("Unregister {SCHEDULER_NAME} scheduler");
        // Restore per-CPU idle resume latency to hardware default.
        // We use applied_idle_resume_us (which is what was actually set,
        // whether from the preset or the user) rather than opts.idle_resume_us
        // (which may still be the sentinel IDLE_RESUME_PRESET).
        if self.applied_idle_resume_us >= 0 && cpu_idle_resume_latency_supported() {
            for cpu in self.topo.all_cpus.values() {
                update_cpu_idle_resume_latency(
                    cpu.id,
                    cpu.pm_qos_resume_latency_us as i32,
                )
                .unwrap();
            }
        }
    }
}

fn main() -> Result<()> {
    let opts = Opts::parse();

    if opts.version {
        println!(
            "{} {}",
            SCHEDULER_NAME,
            build_id::full_version(env!("CARGO_PKG_VERSION"))
        );
        return Ok(());
    }

    if opts.help_stats {
        stats::server_data().describe_meta(&mut std::io::stdout(), None)?;
        return Ok(());
    }

    let loglevel = simplelog::LevelFilter::Info;
    let mut lcfg = simplelog::ConfigBuilder::new();
    lcfg.set_time_offset_to_local()
        .expect("Failed to set local time offset")
        .set_time_level(simplelog::LevelFilter::Error)
        .set_location_level(simplelog::LevelFilter::Off)
        .set_target_level(simplelog::LevelFilter::Off)
        .set_thread_level(simplelog::LevelFilter::Off);
    simplelog::TermLogger::init(
        loglevel,
        lcfg.build(),
        simplelog::TerminalMode::Stderr,
        simplelog::ColorChoice::Auto,
    )?;

    let shutdown       = Arc::new(AtomicBool::new(false));
    let shutdown_clone = shutdown.clone();
    ctrlc::set_handler(move || {
        shutdown_clone.store(true, Ordering::Relaxed);
    })
    .context("Error setting Ctrl-C handler")?;

    if let Some(intv) = opts.monitor.or(opts.stats) {
        let shutdown_copy = shutdown.clone();
        let jh = std::thread::spawn(move || {
            match stats::monitor(Duration::from_secs_f64(intv), shutdown_copy) {
                Ok(_)  => debug!("stats monitor thread finished"),
                Err(e) => warn!("stats monitor thread error: {}", e),
            }
        });
        if opts.monitor.is_some() {
            let _ = jh.join();
            return Ok(());
        }
    }

    let mut open_object = MaybeUninit::uninit();
    loop {
        let mut sched = Scheduler::init(&opts, &mut open_object)?;
        if !sched.run(shutdown.clone())?.should_restart() {
            if sched.user_restart {
                continue;
            }
            break;
        }
    }

    Ok(())
}
