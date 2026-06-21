use std::io::Write;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::Duration;

use anyhow::Result;
use scx_stats::prelude::*;
use scx_stats_derive::stat_doc;
use scx_stats_derive::Stats;
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
    #[stat(desc = "Number of kthread direct dispatches")]
    pub nr_kthread_dispatches: u64,
    #[stat(desc = "Number of task direct dispatches (idle CPU found at enqueue/select_cpu)")]
    pub nr_direct_dispatches: u64,
    #[stat(desc = "Number of default-tier DSQ dispatches")]
    pub nr_shared_dispatches: u64,
    // TIMELY stats (zero when timely mode is disabled)
    #[stat(desc = "Number of delay recovery dispatches")]
    pub nr_delay_recovery_dispatches: u64,
    #[stat(desc = "Number of delay middle add dispatches")]
    pub nr_delay_middle_add_dispatches: u64,
    #[stat(desc = "Number of delay fast recovery dispatches")]
    pub nr_delay_fast_recovery_dispatches: u64,
    #[stat(desc = "Number of delay rate-limited dispatches")]
    pub nr_delay_rate_limited_dispatches: u64,
    #[stat(desc = "Number of gain floor dispatches")]
    pub nr_gain_floor_dispatches: u64,
    #[stat(desc = "Number of gain ceiling dispatches")]
    pub nr_gain_ceiling_dispatches: u64,
    #[stat(desc = "Number of delay low region samples")]
    pub nr_delay_low_region_samples: u64,
    #[stat(desc = "Number of delay mid region samples")]
    pub nr_delay_mid_region_samples: u64,
    #[stat(desc = "Number of delay high region samples")]
    pub nr_delay_high_region_samples: u64,
    #[stat(desc = "Number of gain floor resident samples")]
    pub nr_gain_floor_resident_samples: u64,
    #[stat(desc = "Number of gain mid resident samples")]
    pub nr_gain_mid_resident_samples: u64,
    #[stat(desc = "Number of gain ceiling resident samples")]
    pub nr_gain_ceiling_resident_samples: u64,
    #[stat(desc = "Number of idle select path picks")]
    pub nr_idle_select_path_picks: u64,
    #[stat(desc = "Number of idle enqueue path picks")]
    pub nr_idle_enqueue_path_picks: u64,
    #[stat(desc = "Number of idle prev CPU picks")]
    pub nr_idle_prev_cpu_picks: u64,
    #[stat(desc = "Number of idle primary picks")]
    pub nr_idle_primary_picks: u64,
    #[stat(desc = "Number of idle spill picks")]
    pub nr_idle_spill_picks: u64,
    #[stat(desc = "Number of idle pick failures")]
    pub nr_idle_pick_failures: u64,
    #[stat(desc = "Number of idle primary domain misses")]
    pub nr_idle_primary_domain_misses: u64,
    #[stat(desc = "Number of idle global misses")]
    pub nr_idle_global_misses: u64,
    #[stat(desc = "Number of waker CPU biases")]
    pub nr_waker_cpu_biases: u64,
    #[stat(desc = "Number of keep running reuses")]
    pub nr_keep_running_reuses: u64,
    #[stat(desc = "Number of keep running queue empty")]
    pub nr_keep_running_queue_empty: u64,
    #[stat(desc = "Number of keep running SMT blocked")]
    pub nr_keep_running_smt_blocked: u64,
    #[stat(desc = "Number of keep running queued work")]
    pub nr_keep_running_queued_work: u64,
    #[stat(desc = "Number of dispatch CPU DSQ consumes")]
    pub nr_dispatch_cpu_dsq_consumes: u64,
    #[stat(desc = "Number of dispatch node DSQ consumes (always 0; node_dsq removed)")]
    pub nr_dispatch_node_dsq_consumes: u64,
    #[stat(desc = "Number of CPU release reenqueues")]
    pub nr_cpu_release_reenqueue: u64,
    // ── Laptop / XNU-inspired counters ──────────────────────────────────
    #[stat(desc = "Number of interactive-tier dispatches from the tier DSQ (EDF path)")]
    pub nr_interactive_dispatches: u64,
    #[stat(desc = "Number of background-tier dispatches from the tier DSQ")]
    pub nr_background_dispatches: u64,
    #[stat(desc = "Number of interactive-tier dispatches via the depleting warp budget (ahead of EDF)")]
    pub nr_warp_dispatches: u64,
    #[stat(desc = "Number of default-tier dispatches via the depleting warp budget (ahead of background)")]
    pub nr_default_warp_dispatches: u64,
    #[stat(desc = "Number of process groups promoted to interactive tier by scorer")]
    pub nr_iact_promoted: u64,
    #[stat(desc = "Number of process groups demoted to background tier by scorer")]
    pub nr_iact_demoted: u64,
    #[stat(desc = "Number of times an E-core was skipped to consolidate idle")]
    pub nr_ecore_consolidations: u64,
    #[stat(desc = "Number of times an idle P-core pulled queued work back from an E-core (passive rebalance)")]
    pub nr_ecore_rebalance_pulls: u64,
    #[stat(desc = "Number of SCX_KICK_PREEMPT kicks issued to displace lower-tier tasks (F8)")]
    pub nr_preempt_kicks: u64,
    #[stat(desc = "Number of times WCEL enforcement forced a tier to win dispatch (unconditional, or within an open starvation-avoidance window)")]
    pub nr_wcel_enforcements: u64,
    #[stat(desc = "Number of fresh bounded starvation-avoidance windows opened (a tier was aged-out while a higher tier was also runnable)")]
    pub nr_starvation_window_opens: u64,
}

impl Metrics {
    fn format<W: Write>(&self, w: &mut W) -> Result<()> {
        writeln!(
            w,
            "[{}] r:{:>2}/{:<2} | \
             tiers i:{:<5} d:{:<5} bg:{:<5} | \
             warp:{:<5} dwarp:{:<5} kick:{:<5} wcel:{:<5} starv:{:<4} | \
             score ^:{:<4} v:{:<4} | \
             econs:{:<4} rebal:{:<4} | \
             timely rec:{:<5} mid:{:<5} rl:{:<5} lo:{:<5} hi:{:<5}",
            crate::SCHEDULER_NAME,
            self.nr_running,
            self.nr_cpus,
            self.nr_interactive_dispatches,
            self.nr_shared_dispatches,
            self.nr_background_dispatches,
            self.nr_warp_dispatches,
            self.nr_default_warp_dispatches,
            self.nr_preempt_kicks,
            self.nr_wcel_enforcements,
            self.nr_starvation_window_opens,
            self.nr_iact_promoted,
            self.nr_iact_demoted,
            self.nr_ecore_consolidations,
            self.nr_ecore_rebalance_pulls,
            self.nr_delay_recovery_dispatches,
            self.nr_delay_middle_add_dispatches,
            self.nr_delay_rate_limited_dispatches,
            self.nr_gain_floor_dispatches,
            self.nr_gain_ceiling_dispatches,
        )?;
        Ok(())
    }

    fn delta(&self, rhs: &Self) -> Self {
        Self {
            nr_kthread_dispatches: self.nr_kthread_dispatches
                - rhs.nr_kthread_dispatches,
            nr_direct_dispatches: self.nr_direct_dispatches
                - rhs.nr_direct_dispatches,
            nr_shared_dispatches: self.nr_shared_dispatches
                - rhs.nr_shared_dispatches,
            nr_delay_recovery_dispatches: self.nr_delay_recovery_dispatches
                - rhs.nr_delay_recovery_dispatches,
            nr_delay_middle_add_dispatches: self.nr_delay_middle_add_dispatches
                - rhs.nr_delay_middle_add_dispatches,
            nr_delay_fast_recovery_dispatches: self.nr_delay_fast_recovery_dispatches
                - rhs.nr_delay_fast_recovery_dispatches,
            nr_delay_rate_limited_dispatches: self.nr_delay_rate_limited_dispatches
                - rhs.nr_delay_rate_limited_dispatches,
            nr_gain_floor_dispatches: self.nr_gain_floor_dispatches
                - rhs.nr_gain_floor_dispatches,
            nr_gain_ceiling_dispatches: self.nr_gain_ceiling_dispatches
                - rhs.nr_gain_ceiling_dispatches,
            nr_delay_low_region_samples: self.nr_delay_low_region_samples
                - rhs.nr_delay_low_region_samples,
            nr_delay_mid_region_samples: self.nr_delay_mid_region_samples
                - rhs.nr_delay_mid_region_samples,
            nr_delay_high_region_samples: self.nr_delay_high_region_samples
                - rhs.nr_delay_high_region_samples,
            nr_gain_floor_resident_samples: self.nr_gain_floor_resident_samples
                - rhs.nr_gain_floor_resident_samples,
            nr_gain_mid_resident_samples: self.nr_gain_mid_resident_samples
                - rhs.nr_gain_mid_resident_samples,
            nr_gain_ceiling_resident_samples: self.nr_gain_ceiling_resident_samples
                - rhs.nr_gain_ceiling_resident_samples,
            nr_idle_select_path_picks: self.nr_idle_select_path_picks
                - rhs.nr_idle_select_path_picks,
            nr_idle_enqueue_path_picks: self.nr_idle_enqueue_path_picks
                - rhs.nr_idle_enqueue_path_picks,
            nr_idle_prev_cpu_picks: self.nr_idle_prev_cpu_picks
                - rhs.nr_idle_prev_cpu_picks,
            nr_idle_primary_picks: self.nr_idle_primary_picks
                - rhs.nr_idle_primary_picks,
            nr_idle_spill_picks: self.nr_idle_spill_picks
                - rhs.nr_idle_spill_picks,
            nr_idle_pick_failures: self.nr_idle_pick_failures
                - rhs.nr_idle_pick_failures,
            nr_idle_primary_domain_misses: self.nr_idle_primary_domain_misses
                - rhs.nr_idle_primary_domain_misses,
            nr_idle_global_misses: self.nr_idle_global_misses
                - rhs.nr_idle_global_misses,
            nr_waker_cpu_biases: self.nr_waker_cpu_biases
                - rhs.nr_waker_cpu_biases,
            nr_keep_running_reuses: self.nr_keep_running_reuses
                - rhs.nr_keep_running_reuses,
            nr_keep_running_queue_empty: self.nr_keep_running_queue_empty
                - rhs.nr_keep_running_queue_empty,
            nr_keep_running_smt_blocked: self.nr_keep_running_smt_blocked
                - rhs.nr_keep_running_smt_blocked,
            nr_keep_running_queued_work: self.nr_keep_running_queued_work
                - rhs.nr_keep_running_queued_work,
            nr_dispatch_cpu_dsq_consumes: self.nr_dispatch_cpu_dsq_consumes
                - rhs.nr_dispatch_cpu_dsq_consumes,
            nr_dispatch_node_dsq_consumes: self.nr_dispatch_node_dsq_consumes
                - rhs.nr_dispatch_node_dsq_consumes,
            nr_cpu_release_reenqueue: self.nr_cpu_release_reenqueue
                - rhs.nr_cpu_release_reenqueue,
            // laptop counters
            nr_interactive_dispatches: self.nr_interactive_dispatches
                - rhs.nr_interactive_dispatches,
            nr_background_dispatches: self.nr_background_dispatches
                - rhs.nr_background_dispatches,
            nr_warp_dispatches: self.nr_warp_dispatches
                - rhs.nr_warp_dispatches,
            nr_default_warp_dispatches: self.nr_default_warp_dispatches
                - rhs.nr_default_warp_dispatches,
            nr_iact_promoted: self.nr_iact_promoted - rhs.nr_iact_promoted,
            nr_iact_demoted: self.nr_iact_demoted - rhs.nr_iact_demoted,
            nr_ecore_consolidations: self.nr_ecore_consolidations
                - rhs.nr_ecore_consolidations,
            nr_ecore_rebalance_pulls: self.nr_ecore_rebalance_pulls
                - rhs.nr_ecore_rebalance_pulls,
            nr_preempt_kicks: self.nr_preempt_kicks - rhs.nr_preempt_kicks,
            nr_wcel_enforcements: self.nr_wcel_enforcements - rhs.nr_wcel_enforcements,
            nr_starvation_window_opens: self.nr_starvation_window_opens
                - rhs.nr_starvation_window_opens,
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
