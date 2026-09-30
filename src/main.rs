// SPDX-License-Identifier: GPL-2.0
//
// Copyright (c) 2024 Andrea Righi <andrea.righi@linux.dev>
//
// Laptop-oriented fork of scx_bpfland. The command-line defaults form the
// laptop preset; every value can be overridden.

mod bpf_skel;
pub use bpf_skel::*;
pub mod bpf_intf;
pub use bpf_intf::*;

mod stats;
use std::cmp::Reverse;
use std::ffi::c_int;
use std::fmt::Write;
use std::fs;
use std::mem::MaybeUninit;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::time::Duration;
use std::time::Instant;

use anyhow::Context;
use anyhow::Result;
use anyhow::anyhow;
use anyhow::bail;
use clap::Parser;
use clap::ValueEnum;
use crossbeam::channel::RecvTimeoutError;
use libbpf_rs::OpenObject;
use libbpf_rs::ProgramInput;
use log::{debug, info, warn};
use scx_stats::prelude::*;
use scx_utils::CoreType;
use scx_utils::Cpumask;
use scx_utils::NR_CPU_IDS;
use scx_utils::Powermode;
use scx_utils::Topology;
use scx_utils::UserExitInfo;
use scx_utils::autopower::{PowerProfile, fetch_power_profile};
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
use stats::Metrics;

const SCHEDULER_NAME: &str = "scx_aura";
const MAX_CPUS: usize = 1024;
const POWER_POLL_INTERVAL: Duration = Duration::from_secs(1);
const GOVERNOR_PATH: &str = "/sys/devices/system/cpu/cpufreq/policy0/scaling_governor";

fn cpus_to_cpumask(cpus: &[usize]) -> String {
    if cpus.is_empty() {
        return String::from("none");
    }
    let max_cpu_id = *cpus.iter().max().unwrap();
    let mut bitmask = vec![0u8; (max_cpu_id + 1).div_ceil(8)];
    for cpu_id in cpus {
        bitmask[cpu_id / 8] |= 1 << (cpu_id % 8);
    }
    let hex_str: String = bitmask.iter().rev().fold(String::new(), |mut f, byte| {
        let _ = write!(&mut f, "{:02x}", byte);
        f
    });
    format!("0x{}", hex_str)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
enum PowerMode {
    /// Follow power-profiles-daemon, or the cpufreq energy performance preference.
    Auto,
    Performance,
    Balanced,
    Powersave,
}

/// Power policy pushed to BPF. Frequencies are fractions of a CPU's maximum
/// frequency out of 1024 and only apply with the schedutil governor; 0
/// disables a floor or cap.
struct PowerPolicy {
    iact_floor: u32,
    util_cap: u32,
    bg_cap: u32,
    bg_strict: bool,
    util_prefer_little: bool,
}

impl PowerMode {
    fn resolve(self, profile: PowerProfile) -> PowerMode {
        match self {
            PowerMode::Auto => match profile {
                PowerProfile::Performance => PowerMode::Performance,
                PowerProfile::Powersave => PowerMode::Powersave,
                PowerProfile::Balanced { .. } | PowerProfile::Unknown => PowerMode::Balanced,
            },
            mode => mode,
        }
    }

    fn policy(self) -> PowerPolicy {
        match self {
            PowerMode::Performance => PowerPolicy {
                iact_floor: 1024,
                util_cap: 0,
                bg_cap: 0,
                bg_strict: false,
                util_prefer_little: false,
            },
            PowerMode::Balanced | PowerMode::Auto => PowerPolicy {
                iact_floor: 768,
                util_cap: 0,
                bg_cap: 512,
                bg_strict: false,
                util_prefer_little: false,
            },
            PowerMode::Powersave => PowerPolicy {
                iact_floor: 512,
                util_cap: 768,
                bg_cap: 384,
                bg_strict: true,
                util_prefer_little: true,
            },
        }
    }

    fn index(self) -> u64 {
        match self {
            PowerMode::Performance => 0,
            PowerMode::Balanced | PowerMode::Auto => 1,
            PowerMode::Powersave => 2,
        }
    }
}

/// scx_aura: laptop-oriented sched_ext scheduler with XNU-style QoS tiers.
///
/// Tasks are sorted into interactive, default, utility and background tiers,
/// scheduled with the XNU Clutch root-bucket algorithm (per-tier latency
/// bounds, warp and starvation avoidance). On hybrid CPUs, interactive work
/// prefers performance cores and background work prefers efficiency cores.
/// With the schedutil governor the scheduler also drives CPU frequency.
/// The defaults are the laptop preset.
#[derive(Debug, Parser)]
struct Opts {
    /// Exit debug dump buffer length. 0 indicates default.
    #[clap(long, default_value = "0")]
    exit_dump_len: u32,

    /// Maximum scheduling slice in microseconds. The per-tier latency bounds,
    /// warp budgets and starvation windows scale with it (XNU's values are
    /// defined for a 10 ms quantum).
    #[clap(short = 's', long, default_value = "800")]
    slice_us: u64,

    /// Minimum scheduling slice in microseconds.
    #[clap(short = 'L', long, default_value = "250")]
    slice_min_us: u64,

    /// Maximum time slice lag in microseconds.
    #[clap(short = 'l', long, default_value = "40000")]
    slice_us_lag: u64,

    /// Throttle CPUs by periodically injecting idle cycles (0 = disabled).
    #[clap(short = 't', long, default_value = "0")]
    throttle_us: u64,

    /// Idle QoS resume latency limit in microseconds for the primary domain
    /// (-1 = disabled).
    ///
    /// The limit only excludes idle states whose exit latency is higher.
    /// Deepest core C-states on current laptop CPUs exit in a few hundred
    /// microseconds, so low values cost battery and high ones do nothing.
    #[clap(short = 'I', long, allow_hyphen_values = true, default_value = "-1")]
    idle_resume_us: i64,

    /// Primary scheduling domain: a hex cpumask, or auto, performance,
    /// powersave, turbo, all.
    #[clap(short = 'm', long, default_value = "performance")]
    primary_domain: String,

    /// Power policy (frequency floors and caps, background core confinement).
    /// auto follows power-profiles-daemon, or the cpufreq energy performance
    /// preference when the daemon is not running.
    #[clap(long, value_enum, default_value_t = PowerMode::Auto)]
    power_mode: PowerMode,

    /// Override the detected efficiency (little) cores with a hex cpumask,
    /// or "none".
    #[clap(long)]
    little_cpus: Option<String>,

    /// Enable kthreads prioritization (EXPERIMENTAL).
    #[clap(short = 'k', long, action = clap::ArgAction::SetTrue)]
    local_kthreads: bool,

    /// Ignore synchronous wakeups (kernel idle CPU selection only).
    #[clap(short = 'w', long, action = clap::ArgAction::SetTrue)]
    no_wake_sync: bool,

    /// Disable SMT awareness.
    #[clap(long, action = clap::ArgAction::SetTrue)]
    disable_smt: bool,

    /// Disable NUMA awareness.
    #[clap(long, action = clap::ArgAction::SetTrue)]
    disable_numa: bool,

    /// Disable sticky tasks (tasks averaging under 10 us of CPU time per
    /// wakeup stay on their CPU).
    #[clap(long, action = clap::ArgAction::SetTrue)]
    no_sticky_tasks: bool,

    /// Queue single-CPU tasks in their tier instead of dispatching them
    /// straight to their CPU.
    #[clap(long, action = clap::ArgAction::SetTrue)]
    no_local_pcpu: bool,

    /// Use the kernel's idle CPU selection instead of the capacity-ordered
    /// scan.
    #[clap(long, action = clap::ArgAction::SetTrue)]
    no_preferred_idle_scan: bool,

    /// Do not drive CPU frequency. The cpuperf target then stays at its
    /// kernel default (maximum), which pins schedutil at maximum frequency.
    #[clap(long, action = clap::ArgAction::SetTrue)]
    no_cpufreq: bool,

    /// Disable thread-group interactivity scoring.
    #[clap(long, action = clap::ArgAction::SetTrue)]
    no_group_iact: bool,

    /// Disable core-type placement on hybrid CPUs.
    #[clap(long, alias = "no-ecore-consolidate", action = clap::ArgAction::SetTrue)]
    no_qos_placement: bool,

    /// Disable warp (higher tiers running ahead of the EDF choice).
    #[clap(long, action = clap::ArgAction::SetTrue)]
    no_warp: bool,

    /// Enable TIMELY adaptive time slices.
    #[clap(short = 'T', long, action = clap::ArgAction::SetTrue)]
    timely: bool,

    /// TIMELY low queueing-delay threshold in microseconds.
    #[clap(long, default_value = "5000")]
    timely_tlow_us: u64,

    /// TIMELY high queueing-delay threshold in microseconds.
    #[clap(long, default_value = "50000")]
    timely_thigh_us: u64,

    /// TIMELY minimum slice gain (fixed point, 1024 = 1.0).
    #[clap(long, default_value = "128")]
    timely_gain_min: u32,

    /// TIMELY additive gain step (fixed point).
    #[clap(long, default_value = "32")]
    timely_gain_step: u32,

    /// Consecutive non-rising delay samples before hyperactive increase.
    #[clap(long, default_value = "5")]
    timely_hai_thresh: u32,

    /// Gain step multiplier in hyperactive increase.
    #[clap(long, default_value = "2")]
    timely_hai_multiplier: u32,

    /// TIMELY multiplicative decrease factor (fixed point, 819 = 0.8).
    #[clap(long, default_value = "819")]
    timely_beta: u32,

    /// Delay gradient normalization in microseconds.
    #[clap(long, default_value = "125")]
    timely_gradient_margin_us: u64,

    /// Minimum interval between gain updates of a task, in microseconds.
    #[clap(long, default_value = "500")]
    timely_control_interval_us: u64,

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
    qos_cpus: Vec<usize>,
    power_profile: PowerProfile,
    power_mode: PowerMode,
    last_power_poll: Instant,
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

        info!(
            "{} {} {}",
            SCHEDULER_NAME,
            build_id::full_version(env!("CARGO_PKG_VERSION")),
            if smt_enabled { "SMT on" } else { "SMT off" }
        );
        info!(
            "scheduler options: {}",
            std::env::args().collect::<Vec<_>>().join(" ")
        );

        let power_profile = Self::power_profile();
        let power_mode = opts.power_mode.resolve(power_profile);
        let domain = Self::resolve_energy_domain(&opts.primary_domain, power_profile)
            .map_err(|err| {
                anyhow!(
                    "failed to resolve primary domain '{}': {}",
                    opts.primary_domain,
                    err
                )
            })?;

        let little = Self::little_cpus(opts, &topo)?;
        let hybrid = !opts.no_qos_placement
            && !little.is_empty()
            && little.len() < topo.all_cpus.len();
        info!(
            "little CPUs: {} (core-type placement {})",
            cpus_to_cpumask(&little),
            if hybrid { "on" } else { "off" }
        );

        let mut skel_builder = BpfSkelBuilder::default();
        skel_builder.obj_builder.debug(opts.verbose);
        let open_opts = opts.libbpf.clone().into_bpf_open_opts();
        let mut skel = scx_ops_open!(skel_builder, open_object, aura_ops, open_opts)?;

        skel.struct_ops.aura_ops_mut().exit_dump_len = opts.exit_dump_len;

        let rodata = skel.maps.rodata_data.as_mut().unwrap();
        rodata.debug = opts.debug;
        rodata.smt_enabled = smt_enabled;
        rodata.numa_enabled = numa_enabled;
        rodata.local_pcpu = !opts.no_local_pcpu;
        rodata.no_wake_sync = opts.no_wake_sync;
        rodata.sticky_tasks = !opts.no_sticky_tasks;
        rodata.slice_max = opts.slice_us * 1000;
        rodata.slice_min = opts.slice_min_us.min(opts.slice_us) * 1000;
        rodata.slice_lag = opts.slice_us_lag * 1000;
        rodata.throttle_ns = opts.throttle_us * 1000;
        rodata.primary_all = domain.weight() == *NR_CPU_IDS;
        rodata.preferred_idle_scan = !opts.no_preferred_idle_scan;
        rodata.local_kthreads = opts.local_kthreads || opts.throttle_us > 0;
        rodata.hybrid = hybrid;
        rodata.group_iact_enabled = !opts.no_group_iact;
        rodata.warp_enabled = !opts.no_warp;

        rodata.timely_enabled = opts.timely;
        rodata.timely_tlow_ns = opts.timely_tlow_us * 1000;
        rodata.timely_thigh_ns = opts.timely_thigh_us * 1000;
        rodata.timely_gain_min_fp = opts.timely_gain_min.clamp(1, 1024);
        rodata.timely_gain_step_fp = opts.timely_gain_step;
        rodata.timely_hai_thresh = opts.timely_hai_thresh.max(1);
        rodata.timely_hai_multiplier = opts.timely_hai_multiplier.max(1);
        rodata.timely_beta_fp = opts.timely_beta.min(1024);
        rodata.timely_gradient_margin_ns = opts.timely_gradient_margin_us.max(1) * 1000;
        rodata.timely_control_interval_ns = opts.timely_control_interval_us * 1000;

        // Big cores first, then by capacity: the BPF side scans this list
        // forward for performance placement and backward for efficiency.
        let mut cpus: Vec<_> = topo.all_cpus.values().collect();
        cpus.sort_by_key(|cpu| (little.contains(&cpu.id), Reverse(cpu.cpu_capacity), cpu.id));
        cpus.truncate(MAX_CPUS);
        for (i, cpu) in cpus.iter().enumerate() {
            rodata.preferred_cpus[i] = cpu.id as u64;
            if cpu.id < MAX_CPUS && little.contains(&cpu.id) {
                rodata.cpu_little[cpu.id] = 1;
            }
        }
        rodata.nr_preferred = cpus.len() as u32;
        info!(
            "CPU scan order: {:?}",
            cpus.iter().map(|cpu| cpu.id).collect::<Vec<_>>()
        );

        // ops.cpu_release() is deprecated once scx_bpf_reenqueue_local() can be
        // called from any context; use the sched_switch hook there instead.
        if scx_utils::ksym_exists("scx_bpf_reenqueue_local___v2").unwrap_or(false) {
            skel.struct_ops.aura_ops_mut().cpu_release = std::ptr::null_mut();
            skel.progs.aura_sched_switch.set_autoload(true);
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
        info!(
            "scheduler flags: {:#x}",
            skel.struct_ops.aura_ops_mut().flags
        );

        let mut skel = scx_ops_load!(skel, aura_ops, uei)?;

        Self::init_energy_domain(&mut skel, &domain).map_err(|err| {
            anyhow!("failed to initialize primary domain 0x{:x}: {}", domain, err)
        })?;

        skel.maps.bss_data.as_mut().unwrap().cpufreq_perf_lvl =
            if opts.no_cpufreq { 1024 } else { -1 };
        Self::report_cpufreq(opts);
        Self::apply_power_policy(&mut skel, power_mode);
        info!("power mode: {:?} (profile: {})", power_mode, power_profile);

        if smt_enabled {
            Self::init_smt_domains(&mut skel, &topo)?;
        }

        let struct_ops = Some(scx_ops_attach!(skel, aura_ops)?);
        let stats_server = StatsServer::new(stats::server_data()).launch()?;

        // Last, so that no earlier failure can leave the limit applied.
        let qos_cpus = Self::apply_idle_qos(opts, &topo, &domain)?;

        Ok(Self {
            skel,
            struct_ops,
            opts,
            topo,
            qos_cpus,
            power_profile,
            power_mode,
            last_power_poll: Instant::now(),
            stats_server,
            user_restart: false,
        })
    }

    fn apply_idle_qos(opts: &Opts, topo: &Topology, domain: &Cpumask) -> Result<Vec<usize>> {
        let mut applied = Vec::new();
        if opts.idle_resume_us < 0 {
            return Ok(applied);
        }
        if !cpu_idle_resume_latency_supported() {
            warn!("idle resume latency QoS not supported");
            return Ok(applied);
        }
        let limit: i32 = opts.idle_resume_us.try_into()?;
        for cpu in topo.all_cpus.values().filter(|cpu| domain.test_cpu(cpu.id)) {
            if let Err(err) = update_cpu_idle_resume_latency(cpu.id, limit) {
                Self::restore_idle_qos(topo, &applied);
                return Err(err);
            }
            applied.push(cpu.id);
        }
        info!(
            "idle resume latency limited to {} us on {}",
            limit,
            cpus_to_cpumask(&applied)
        );
        Ok(applied)
    }

    fn restore_idle_qos(topo: &Topology, cpus: &[usize]) {
        for &cpu in cpus {
            let Some(orig) = topo.all_cpus.get(&cpu) else {
                continue;
            };
            if let Err(err) =
                update_cpu_idle_resume_latency(cpu, orig.pm_qos_resume_latency_us as i32)
            {
                warn!("failed to restore idle resume latency of CPU {}: {}", cpu, err);
            }
        }
    }

    fn little_cpus(opts: &Opts, topo: &Topology) -> Result<Vec<usize>> {
        match opts.little_cpus.as_deref() {
            Some("none") => Ok(Vec::new()),
            Some(mask) => {
                let mask = Cpumask::from_str(mask)?;
                Ok(topo
                    .all_cpus
                    .keys()
                    .copied()
                    .filter(|&cpu| mask.test_cpu(cpu))
                    .collect())
            }
            None => Ok(topo
                .all_cpus
                .values()
                .filter(|cpu| cpu.core_type == CoreType::Little)
                .map(|cpu| cpu.id)
                .collect()),
        }
    }

    fn apply_power_policy(skel: &mut BpfSkel<'_>, mode: PowerMode) {
        let policy = mode.policy();
        let bss = skel.maps.bss_data.as_mut().unwrap();
        bss.perf_iact_floor = policy.iact_floor;
        bss.perf_util_cap = policy.util_cap;
        bss.perf_bg_cap = policy.bg_cap;
        bss.bg_strict = policy.bg_strict;
        bss.util_prefer_little = policy.util_prefer_little;
    }

    fn report_cpufreq(opts: &Opts) {
        if opts.no_cpufreq {
            info!("cpufreq: disabled, cpuperf target left at maximum");
            return;
        }
        match fs::read_to_string(GOVERNOR_PATH) {
            Ok(gov) if gov.trim() == "schedutil" => info!("cpufreq: driving schedutil"),
            Ok(gov) => info!(
                "cpufreq: governor is {}; frequency targets only apply with schedutil",
                gov.trim()
            ),
            Err(_) => info!("cpufreq: no cpufreq policy found"),
        }
    }

    fn enable_primary_cpu(skel: &mut BpfSkel<'_>, cpu: i32) -> Result<(), u32> {
        let prog = &mut skel.progs.enable_primary_cpu;
        let mut args = cpu_arg {
            cpu_id: cpu as c_int,
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

    fn epp_to_cpumask(profile: Powermode) -> Result<Cpumask> {
        let mut cpus = get_primary_cpus(profile).unwrap_or_default();
        if cpus.is_empty() {
            cpus = get_primary_cpus(Powermode::Any).unwrap_or_default();
        }
        Cpumask::from_str(&cpus_to_cpumask(&cpus))
    }

    fn resolve_energy_domain(primary_domain: &str, power_profile: PowerProfile) -> Result<Cpumask> {
        let domain = match primary_domain {
            "powersave" => Self::epp_to_cpumask(Powermode::Powersave)?,
            "performance" => Self::epp_to_cpumask(Powermode::Performance)?,
            "turbo" => Self::epp_to_cpumask(Powermode::Turbo)?,
            "auto" => match power_profile {
                PowerProfile::Powersave => Self::epp_to_cpumask(Powermode::Powersave)?,
                PowerProfile::Balanced { .. }
                | PowerProfile::Performance
                | PowerProfile::Unknown => Self::epp_to_cpumask(Powermode::Any)?,
            },
            "all" => Self::epp_to_cpumask(Powermode::Any)?,
            &_ => Cpumask::from_str(primary_domain)?,
        };
        Ok(domain)
    }

    fn init_energy_domain(skel: &mut BpfSkel<'_>, domain: &Cpumask) -> Result<()> {
        info!("primary CPU domain = 0x{:x}", domain);
        if let Err(err) = Self::enable_primary_cpu(skel, -1) {
            bail!("failed to reset primary domain: error {}", err);
        }
        for cpu in 0..*NR_CPU_IDS {
            if !domain.test_cpu(cpu) {
                continue;
            }
            if let Err(err) = Self::enable_primary_cpu(skel, cpu as i32) {
                bail!("failed to add CPU {} to primary domain: error {}", cpu, err);
            }
        }
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

    fn init_smt_domains(skel: &mut BpfSkel<'_>, topo: &Topology) -> Result<()> {
        let smt_siblings = topo.sibling_cpus();
        info!("SMT sibling CPUs: {:?}", smt_siblings);
        for (cpu, &sibling) in smt_siblings.iter().enumerate() {
            if sibling < 0 {
                continue;
            }
            Self::enable_sibling_cpu(skel, cpu, sibling as usize).map_err(|err| {
                anyhow!("failed to set SMT sibling of CPU {}: error {}", cpu, err)
            })?;
        }
        Ok(())
    }

    fn get_metrics(&self) -> Metrics {
        let bss = self.skel.maps.bss_data.as_ref().unwrap();
        let tier = |counters: &[u64], t: aura_tier| counters[t as usize];
        let running = |t: aura_tier| bss.nr_tier_running[t as usize].max(0) as u64;
        Metrics {
            nr_running: bss.nr_running,
            nr_cpus: bss.nr_online_cpus,
            power_mode: self.power_mode.index(),
            cpus_iact: running(aura_tier_TIER_INTERACTIVE),
            cpus_def: running(aura_tier_TIER_DEFAULT),
            cpus_util: running(aura_tier_TIER_UTILITY),
            cpus_bg: running(aura_tier_TIER_BACKGROUND),
            nr_direct_dispatches: bss.nr_direct_dispatches,
            nr_local_dispatches: bss.nr_local_dispatches,
            nr_keep_running: bss.nr_keep_running,
            nr_iact_enqueues: tier(&bss.nr_tier_enqueues, aura_tier_TIER_INTERACTIVE),
            nr_def_enqueues: tier(&bss.nr_tier_enqueues, aura_tier_TIER_DEFAULT),
            nr_util_enqueues: tier(&bss.nr_tier_enqueues, aura_tier_TIER_UTILITY),
            nr_bg_enqueues: tier(&bss.nr_tier_enqueues, aura_tier_TIER_BACKGROUND),
            nr_iact_dispatches: tier(&bss.nr_tier_dispatches, aura_tier_TIER_INTERACTIVE),
            nr_def_dispatches: tier(&bss.nr_tier_dispatches, aura_tier_TIER_DEFAULT),
            nr_util_dispatches: tier(&bss.nr_tier_dispatches, aura_tier_TIER_UTILITY),
            nr_bg_dispatches: tier(&bss.nr_tier_dispatches, aura_tier_TIER_BACKGROUND),
            nr_sel_natural: bss.nr_sel_natural,
            nr_sel_warp: bss.nr_sel_warp,
            nr_sel_starve_open: bss.nr_sel_starve_open,
            nr_sel_starve: bss.nr_sel_starve,
            nr_preempt_kicks: bss.nr_preempt_kicks,
            nr_mig_up: bss.nr_mig_up,
            nr_mig_down: bss.nr_mig_down,
            nr_mig_smt: bss.nr_mig_smt,
            nr_promotions: bss.nr_promotions,
            nr_demotions: bss.nr_demotions,
            nr_reenq: bss.nr_reenq,
            nr_timely_inc: bss.nr_timely_inc,
            nr_timely_dec: bss.nr_timely_dec,
            nr_timely_hai: bss.nr_timely_hai,
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

    /// Track power profile changes. Returns true when the scheduler must be
    /// restarted, which is only needed when the primary domain follows the
    /// profile; everything else is updated in place.
    fn refresh_power(&mut self) -> bool {
        let auto_domain = self.opts.primary_domain == "auto";
        if self.opts.power_mode != PowerMode::Auto && !auto_domain {
            return false;
        }
        if self.last_power_poll.elapsed() < POWER_POLL_INTERVAL {
            return false;
        }
        self.last_power_poll = Instant::now();

        let profile = Self::power_profile();
        if profile == self.power_profile {
            return false;
        }
        info!("power profile: {} -> {}", self.power_profile, profile);
        self.power_profile = profile;
        if auto_domain {
            return true;
        }

        let mode = self.opts.power_mode.resolve(profile);
        if mode != self.power_mode {
            self.power_mode = mode;
            Self::apply_power_policy(&mut self.skel, mode);
            info!("power mode: {:?}", mode);
        }
        false
    }

    fn run(&mut self, shutdown: Arc<AtomicBool>) -> Result<UserExitInfo> {
        let (res_ch, req_ch) = self.stats_server.channels();
        while !shutdown.load(Ordering::Relaxed) && !self.exited() {
            if self.refresh_power() {
                self.user_restart = true;
                break;
            }
            match req_ch.recv_timeout(Duration::from_secs(1)) {
                Ok(()) => res_ch.send(self.get_metrics())?,
                Err(RecvTimeoutError::Timeout) => {}
                Err(e) => Err(e)?,
            }
        }
        let _ = self.struct_ops.take();
        uei_report!(&self.skel, uei)
    }
}

impl Drop for Scheduler<'_> {
    fn drop(&mut self) {
        info!("Unregister {SCHEDULER_NAME} scheduler");
        Self::restore_idle_qos(&self.topo, &self.qos_cpus);
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

    let shutdown = Arc::new(AtomicBool::new(false));
    let shutdown_clone = shutdown.clone();
    ctrlc::set_handler(move || {
        shutdown_clone.store(true, Ordering::Relaxed);
    })
    .context("Error setting Ctrl-C handler")?;

    if let Some(intv) = opts.monitor.or(opts.stats) {
        let shutdown_copy = shutdown.clone();
        let jh = std::thread::spawn(move || {
            match stats::monitor(Duration::from_secs_f64(intv), shutdown_copy) {
                Ok(_) => debug!("stats monitor thread finished"),
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
