use std::io::Write;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::time::Duration;

use anyhow::Result;
use scx_stats::prelude::*;
use scx_stats_derive::Stats;
use scx_stats_derive::stat_doc;
use serde::Deserialize;
use serde::Serialize;

#[stat_doc]
#[derive(Clone, Debug, Default, Serialize, Deserialize, Stats)]
#[stat(top)]
pub struct Metrics {
    #[stat(desc = "Number of running tasks")]
    pub nr_running: u64,
    #[stat(desc = "Number of online CPUs")]
    pub nr_cpus: u64,
    #[stat(desc = "Active power mode (0 = performance, 1 = balanced, 2 = powersave)")]
    pub power_mode: u64,
    #[stat(desc = "CPUs running an interactive-tier task")]
    pub cpus_iact: u64,
    #[stat(desc = "CPUs running a default-tier task")]
    pub cpus_def: u64,
    #[stat(desc = "CPUs running a utility-tier task")]
    pub cpus_util: u64,
    #[stat(desc = "CPUs running a background-tier task")]
    pub cpus_bg: u64,
    #[stat(desc = "Tasks placed directly on an idle CPU")]
    pub nr_direct_dispatches: u64,
    #[stat(desc = "Tasks inserted into their CPU's local DSQ: sticky and per-CPU tasks, and tasks resuming after a preemption or a lower tier's turn")]
    pub nr_local_dispatches: u64,
    #[stat(desc = "Slice refills for a task that kept its CPU")]
    pub nr_keep_running: u64,
    #[stat(desc = "Enqueues into the interactive tier queue (direct preemptions are counted in nr_preempt_kicks)")]
    pub nr_iact_enqueues: u64,
    #[stat(desc = "Enqueues into the default tier")]
    pub nr_def_enqueues: u64,
    #[stat(desc = "Enqueues into the utility tier")]
    pub nr_util_enqueues: u64,
    #[stat(desc = "Enqueues into the background tier")]
    pub nr_bg_enqueues: u64,
    #[stat(desc = "Dispatches from the interactive tier")]
    pub nr_iact_dispatches: u64,
    #[stat(desc = "Dispatches from the default tier")]
    pub nr_def_dispatches: u64,
    #[stat(desc = "Dispatches from the utility tier")]
    pub nr_util_dispatches: u64,
    #[stat(desc = "Dispatches from the background tier")]
    pub nr_bg_dispatches: u64,
    #[stat(desc = "Root bucket selections in natural (EDF and priority) order")]
    pub nr_sel_natural: u64,
    #[stat(desc = "Root bucket selections of a higher tier running ahead of EDF on warp")]
    pub nr_sel_warp: u64,
    #[stat(desc = "Starvation-avoidance windows opened for a lower tier")]
    pub nr_sel_starve_open: u64,
    #[stat(desc = "Selections made inside an open starvation-avoidance window")]
    pub nr_sel_starve: u64,
    #[stat(desc = "CPUs preempted to run an interactive task")]
    pub nr_preempt_kicks: u64,
    #[stat(desc = "Quantum-expiry migrations from a little to a big core, including swaps with little-core work running on a big core")]
    pub nr_mig_up: u64,
    #[stat(desc = "Quantum-expiry migrations from a big to a little core")]
    pub nr_mig_down: u64,
    #[stat(desc = "Quantum-expiry migrations away from a busy SMT sibling")]
    pub nr_mig_smt: u64,
    #[stat(desc = "Tasks moved to a higher tier")]
    pub nr_promotions: u64,
    #[stat(desc = "Tasks moved to a lower tier")]
    pub nr_demotions: u64,
    #[stat(desc = "Tasks re-enqueued after their CPU was taken by a higher sched class")]
    pub nr_reenq: u64,
    #[stat(desc = "TIMELY additive gain increases")]
    pub nr_timely_inc: u64,
    #[stat(desc = "TIMELY multiplicative gain decreases")]
    pub nr_timely_dec: u64,
    #[stat(desc = "TIMELY hyperactive increases")]
    pub nr_timely_hai: u64,
}

impl Metrics {
    fn format<W: Write>(&self, w: &mut W) -> Result<()> {
        let mode = match self.power_mode {
            0 => "perf",
            1 => "bal",
            _ => "save",
        };
        writeln!(
            w,
            "[{}] r:{:>2}/{:<2} {} run i{} d{} u{} b{} | enq i:{:<5} d:{:<5} u:{:<5} b:{:<5} | \
             disp i:{:<5} d:{:<5} u:{:<5} b:{:<5} | sel nat:{:<5} warp:{:<4} \
             starve:{:<3}/{:<4} | kick:{:<4} mig ^{:<4} v{:<4} smt:{:<4} | \
             tier ^{:<4} v{:<4} | keep:{:<5} direct:{:<5} local:{:<5}",
            crate::SCHEDULER_NAME,
            self.nr_running,
            self.nr_cpus,
            mode,
            self.cpus_iact,
            self.cpus_def,
            self.cpus_util,
            self.cpus_bg,
            self.nr_iact_enqueues,
            self.nr_def_enqueues,
            self.nr_util_enqueues,
            self.nr_bg_enqueues,
            self.nr_iact_dispatches,
            self.nr_def_dispatches,
            self.nr_util_dispatches,
            self.nr_bg_dispatches,
            self.nr_sel_natural,
            self.nr_sel_warp,
            self.nr_sel_starve_open,
            self.nr_sel_starve,
            self.nr_preempt_kicks,
            self.nr_mig_up,
            self.nr_mig_down,
            self.nr_mig_smt,
            self.nr_promotions,
            self.nr_demotions,
            self.nr_keep_running,
            self.nr_direct_dispatches,
            self.nr_local_dispatches,
        )?;
        if self.nr_timely_inc + self.nr_timely_dec + self.nr_timely_hai > 0 {
            writeln!(
                w,
                "[{}] timely inc:{:<5} dec:{:<5} hai:{:<5}",
                crate::SCHEDULER_NAME,
                self.nr_timely_inc,
                self.nr_timely_dec,
                self.nr_timely_hai,
            )?;
        }
        Ok(())
    }

    fn delta(&self, rhs: &Self) -> Self {
        Self {
            nr_direct_dispatches: self.nr_direct_dispatches - rhs.nr_direct_dispatches,
            nr_local_dispatches: self.nr_local_dispatches - rhs.nr_local_dispatches,
            nr_keep_running: self.nr_keep_running - rhs.nr_keep_running,
            nr_iact_enqueues: self.nr_iact_enqueues - rhs.nr_iact_enqueues,
            nr_def_enqueues: self.nr_def_enqueues - rhs.nr_def_enqueues,
            nr_util_enqueues: self.nr_util_enqueues - rhs.nr_util_enqueues,
            nr_bg_enqueues: self.nr_bg_enqueues - rhs.nr_bg_enqueues,
            nr_iact_dispatches: self.nr_iact_dispatches - rhs.nr_iact_dispatches,
            nr_def_dispatches: self.nr_def_dispatches - rhs.nr_def_dispatches,
            nr_util_dispatches: self.nr_util_dispatches - rhs.nr_util_dispatches,
            nr_bg_dispatches: self.nr_bg_dispatches - rhs.nr_bg_dispatches,
            nr_sel_natural: self.nr_sel_natural - rhs.nr_sel_natural,
            nr_sel_warp: self.nr_sel_warp - rhs.nr_sel_warp,
            nr_sel_starve_open: self.nr_sel_starve_open - rhs.nr_sel_starve_open,
            nr_sel_starve: self.nr_sel_starve - rhs.nr_sel_starve,
            nr_preempt_kicks: self.nr_preempt_kicks - rhs.nr_preempt_kicks,
            nr_mig_up: self.nr_mig_up - rhs.nr_mig_up,
            nr_mig_down: self.nr_mig_down - rhs.nr_mig_down,
            nr_mig_smt: self.nr_mig_smt - rhs.nr_mig_smt,
            nr_promotions: self.nr_promotions - rhs.nr_promotions,
            nr_demotions: self.nr_demotions - rhs.nr_demotions,
            nr_reenq: self.nr_reenq - rhs.nr_reenq,
            nr_timely_inc: self.nr_timely_inc - rhs.nr_timely_inc,
            nr_timely_dec: self.nr_timely_dec - rhs.nr_timely_dec,
            nr_timely_hai: self.nr_timely_hai - rhs.nr_timely_hai,
            ..self.clone()
        }
    }
}

pub fn server_data() -> StatsServerData<(), Metrics> {
    let open: Box<dyn StatsOpener<(), Metrics>> = Box::new(move |(req_ch, res_ch)| {
        req_ch.send(())?;
        let mut prev = res_ch.recv()?;

        let read: Box<dyn StatsReader<(), Metrics>> =
            Box::new(move |_args, (req_ch, res_ch)| {
                req_ch.send(())?;
                let cur = res_ch.recv()?;
                let delta = cur.delta(&prev);
                prev = cur;
                delta.to_json()
            });

        Ok(read)
    });

    StatsServerData::new()
        .add_meta(Metrics::meta())
        .add_ops("top", StatsOps { open, close: None })
}

pub fn monitor(intv: Duration, shutdown: Arc<AtomicBool>) -> Result<()> {
    scx_utils::monitor_stats::<Metrics>(
        &[],
        intv,
        || shutdown.load(Ordering::Relaxed),
        |metrics| metrics.format(&mut std::io::stdout()),
    )
}
