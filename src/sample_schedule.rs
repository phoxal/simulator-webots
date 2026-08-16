//! Logical-time publish schedules for simulated capabilities.
//!
//! This is the framework's `phoxal::SampleSchedule`, carried across with the
//! controller. It lives here rather than being imported because it is the one
//! piece of the facade this binary still needs, and depending on `phoxal` for
//! it would drag in the participant runner, the role attributes and the setup
//! context - the whole supervised-participant machinery this process
//! deliberately is not. The rule it encodes is small and stable: a capability
//! declares a publish rate, the controller resolves it to a logical-time
//! deadline schedule once at setup, and every family applies it the same way.

use std::num::NonZeroU64;

use anyhow::Result;

/// The one policy used when a producer reaches a deadline after one or more
/// deadlines have already passed.
///
/// A producer emits at most one sample for a step and advances the phase to
/// the first deadline after that step. It does not emit a burst of catch-up
/// samples, because those samples would all describe the same observation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum MissedTickPolicy {
    /// Skip missed deadlines and keep the schedule phase on logical time.
    Skip,
}

/// A publish cadence expressed as nanosecond deadlines on a logical timeline.
///
/// The schedule keeps its phase instead of reducing a requested rate to an
/// integer step divisor. For example, a 30 Hz schedule on a 100 Hz source has
/// deadlines at approximately 0, 33.3, 66.7, 100, ... milliseconds, producing
/// 4/3/3-step spacing (and the corresponding 3/3/4 rotation) over time.
#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct SampleSchedule {
    period_ns: NonZeroU64,
    anchor_ns: u64,
    next_deadline_ns: u64,
    exhausted: bool,
}

impl SampleSchedule {
    /// The missed-deadline policy shared by every producer.
    pub(crate) const MISSED_TICK_POLICY: MissedTickPolicy = MissedTickPolicy::Skip;

    /// Validate a requested cadence and build its logical-time schedule.
    ///
    /// `step_hz` is the source cadence. A producer cannot publish faster than
    /// the source can provide observations, and both rates must be finite and
    /// positive. Rates are converted to nanoseconds explicitly, with a
    /// checked, nearest-nanosecond period rather than a floating-point divisor
    /// retained for use in the step loop.
    #[cfg_attr(
        not(test),
        expect(
            dead_code,
            reason = "\
        the rate-pair constructor is the shape the framework's shared schedule \
        offers and the shape the battery's tests build a spec with; every \
        capability in this binary reaches a schedule through \
        `from_source_period_ns`, because a Webots device's cadence is a \
        quantized millisecond period rather than a rate"
        )
    )]
    pub(crate) fn new(capability_id: &str, step_hz: f64, publish_rate_hz: f64) -> Result<Self> {
        validate_rate(capability_id, "step_hz", step_hz)?;
        validate_rate(capability_id, "publish_rate_hz", publish_rate_hz)?;
        let source_period_ns = period_ns(capability_id, "step_hz", step_hz)?;
        let publish_period_ns = period_ns(capability_id, "publish_rate_hz", publish_rate_hz)?;
        if publish_rate_hz > step_hz || publish_period_ns.get() < source_period_ns.get() {
            anyhow::bail!(
                "capability '{capability_id}' publish_rate_hz ({publish_rate_hz}) must not exceed source step_hz ({step_hz})"
            );
        }
        Ok(Self::from_periods(publish_period_ns))
    }

    /// Build a schedule from the source's effective period in nanoseconds.
    ///
    /// This is what a Webots device needs: its requested millisecond period was
    /// quantized to the world's basic time step, so its cadence is not
    /// represented by a floating point rate. When that effective period is
    /// slower than the requested publish cadence, the schedule uses the source
    /// period rather than repeating one observation.
    pub(crate) fn from_source_period_ns(
        capability_id: &str,
        source_period_ns: u64,
        publish_rate_hz: f64,
    ) -> Result<Self> {
        if source_period_ns == 0 {
            anyhow::bail!("capability '{capability_id}' source period must be > 0 ns");
        }
        validate_rate(capability_id, "publish_rate_hz", publish_rate_hz)?;
        let publish_period_ns = period_ns(capability_id, "publish_rate_hz", publish_rate_hz)?;
        let effective_period_ns = publish_period_ns.get().max(source_period_ns);
        let effective_period_ns = NonZeroU64::new(effective_period_ns)
            .ok_or_else(|| anyhow::anyhow!("capability '{capability_id}' period must be > 0 ns"))?;
        Ok(Self::from_periods(effective_period_ns))
    }

    const fn from_periods(period_ns: NonZeroU64) -> Self {
        Self {
            period_ns,
            anchor_ns: 0,
            next_deadline_ns: 0,
            exhausted: false,
        }
    }

    /// The effective publish cadence in the framework's nanosecond time domain.
    pub(crate) const fn period_ns(&self) -> u64 {
        self.period_ns.get()
    }

    /// The one missed-deadline policy all schedules apply.
    #[cfg_attr(
        not(test),
        expect(
            dead_code,
            reason = "\
        the policy is part of the schedule's stated contract and is asserted by \
        its tests; the step loop applies it rather than querying it"
        )
    )]
    pub(crate) const fn missed_tick_policy(&self) -> MissedTickPolicy {
        Self::MISSED_TICK_POLICY
    }

    /// Re-anchor the first deadline at `logical_time_ns`.
    ///
    /// Call this when a producer starts a new logical timeline. The first
    /// source step at the anchor is due, and subsequent deadlines retain the
    /// requested phase regardless of the source step's integer cadence.
    pub(crate) const fn reanchor(&mut self, logical_time_ns: u64) {
        self.anchor_ns = logical_time_ns;
        self.next_deadline_ns = logical_time_ns;
        self.exhausted = false;
    }

    /// Re-anchor with the first deadline after a source refresh delay.
    pub(crate) fn reanchor_after(&mut self, logical_time_ns: u64, delay_ns: u64) -> Result<()> {
        let first_deadline = logical_time_ns
            .checked_add(delay_ns)
            .ok_or_else(|| anyhow::anyhow!("sample schedule exhausted its logical-time range"))?;
        self.reanchor(first_deadline);
        Ok(())
    }

    /// Re-anchor at the beginning of a logical timeline.
    #[cfg_attr(
        not(test),
        expect(
            dead_code,
            reason = "\
        a rewound Webots world re-anchors at the instant it rewound to, through \
        `reanchor_after`, never at zero; the zero case is exercised by this \
        module's tests"
        )
    )]
    pub(crate) const fn reset(&mut self) {
        self.reanchor(0);
    }

    /// Whether a sample is due at `logical_time_ns`, advancing this schedule's
    /// phase when it is.
    ///
    /// Under [`MissedTickPolicy::Skip`], a late call emits one sample and
    /// jumps directly to the first deadline after the current logical time.
    #[must_use = "the result reports whether the schedule is due"]
    pub(crate) fn is_due_at(&mut self, logical_time_ns: u64) -> Result<bool> {
        if self.exhausted {
            anyhow::bail!("sample schedule exhausted its logical-time range");
        }
        if logical_time_ns < self.next_deadline_ns {
            return Ok(false);
        }

        let elapsed_ns = logical_time_ns
            .checked_sub(self.anchor_ns)
            .ok_or_else(|| anyhow::anyhow!("sample schedule logical time moved backwards"))?;
        let periods_elapsed = elapsed_ns / self.period_ns.get();
        let next_period = periods_elapsed.checked_add(1).ok_or_else(|| {
            self.exhausted = true;
            anyhow::anyhow!("sample schedule exhausted its logical-time range")
        })?;
        let offset_ns = next_period
            .checked_mul(self.period_ns.get())
            .ok_or_else(|| {
                self.exhausted = true;
                anyhow::anyhow!("sample schedule exhausted its logical-time range")
            })?;
        self.next_deadline_ns = self.anchor_ns.checked_add(offset_ns).ok_or_else(|| {
            self.exhausted = true;
            anyhow::anyhow!("sample schedule exhausted its logical-time range")
        })?;
        Ok(true)
    }
}

fn validate_rate(capability_id: &str, name: &str, rate_hz: f64) -> Result<()> {
    if !rate_hz.is_finite() || rate_hz <= 0.0 {
        anyhow::bail!("capability '{capability_id}' {name} must be finite and > 0");
    }
    Ok(())
}

fn period_ns(capability_id: &str, name: &str, rate_hz: f64) -> Result<NonZeroU64> {
    const NANOS_PER_SECOND: f64 = 1_000_000_000.0;
    let period = NANOS_PER_SECOND / rate_hz;
    if !period.is_finite() || period > u64::MAX as f64 {
        anyhow::bail!("capability '{capability_id}' {name} period does not fit in nanoseconds");
    }

    // The rate is validated above, and a positive finite period rounds to at
    // least one nanosecond. The explicit bound keeps the float-to-integer cast
    // from silently saturating at the edge of the integer domain.
    let rounded = period.round();
    let period_ns = u64::try_from(rounded as u128).map_err(|_| {
        anyhow::anyhow!("capability '{capability_id}' {name} period does not fit in nanoseconds")
    })?;
    NonZeroU64::new(period_ns).ok_or_else(|| {
        anyhow::anyhow!("capability '{capability_id}' {name} period must be at least 1 ns")
    })
}

#[cfg(test)]
mod tests {
    use super::{MissedTickPolicy, SampleSchedule};

    #[test]
    fn a_rate_at_the_source_cadence_publishes_on_every_source_step() {
        let mut schedule = SampleSchedule::new("imu", 100.0, 100.0).unwrap();
        assert_eq!(schedule.period_ns(), 10_000_000);
        assert!((0..5).all(|step| schedule.is_due_at(step * 10_000_000).unwrap()));
    }

    #[test]
    fn a_slower_exact_rate_keeps_its_nanosecond_phase() {
        let mut schedule = SampleSchedule::new("range", 100.0, 25.0).unwrap();
        assert_eq!(schedule.period_ns(), 40_000_000);
        assert_eq!(
            (0..9)
                .map(|step| schedule.is_due_at(step * 10_000_000).unwrap())
                .collect::<Vec<_>>(),
            [true, false, false, false, true, false, false, false, true]
        );
    }

    #[test]
    fn thirty_hz_on_a_hundred_hz_source_does_not_round_to_a_divisor() {
        let mut schedule = SampleSchedule::new("gnss", 100.0, 30.0).unwrap();
        assert_eq!(schedule.period_ns(), 33_333_333);

        let due_steps = (0..31)
            .filter(|step| schedule.is_due_at(*step * 10_000_000).unwrap())
            .collect::<Vec<_>>();
        assert_eq!(due_steps, [0, 4, 7, 10, 14, 17, 20, 24, 27, 30]);
    }

    #[test]
    fn common_rates_keep_their_long_run_cadence() {
        for (rate_hz, expected_samples) in [(10.0, 101), (20.0, 201), (30.0, 301), (50.0, 501)] {
            let mut schedule = SampleSchedule::new("camera", 100.0, rate_hz).unwrap();
            let samples = (0..=1_000)
                .filter(|step| schedule.is_due_at(*step * 10_000_000).unwrap())
                .count();
            assert_eq!(samples, expected_samples, "{rate_hz} Hz");
        }
    }

    #[test]
    fn missed_deadlines_are_skipped_under_the_shared_policy() {
        let mut schedule = SampleSchedule::new("camera", 100.0, 20.0).unwrap();
        assert_eq!(schedule.missed_tick_policy(), MissedTickPolicy::Skip);
        assert!(schedule.is_due_at(0).unwrap());
        // The 50 ms and 100 ms deadlines were missed; one observation is
        // emitted now, and the phase resumes at 150 ms.
        assert!(schedule.is_due_at(120_000_000).unwrap());
        assert!(!schedule.is_due_at(130_000_000).unwrap());
        assert!(schedule.is_due_at(150_000_000).unwrap());
    }

    #[test]
    fn reset_and_reanchor_start_a_fresh_phase() {
        let mut schedule = SampleSchedule::new("imu", 100.0, 25.0).unwrap();
        assert!(schedule.is_due_at(0).unwrap());
        assert!(!schedule.is_due_at(10_000_000).unwrap());

        schedule.reanchor(100_000_000);
        assert!(!schedule.is_due_at(90_000_000).unwrap());
        assert!(schedule.is_due_at(100_000_000).unwrap());
        assert!(!schedule.is_due_at(110_000_000).unwrap());
        assert!(schedule.is_due_at(140_000_000).unwrap());

        schedule.reset();
        assert!(schedule.is_due_at(0).unwrap());
    }

    #[test]
    fn exhausted_deadline_is_a_checked_terminal_error_and_reset_recovers() {
        let mut schedule = SampleSchedule::new("camera", 100.0, 100.0).unwrap();
        schedule.reanchor(u64::MAX);
        let error = schedule
            .is_due_at(u64::MAX)
            .expect_err("advancing beyond u64 nanoseconds must be checked");
        assert_eq!(
            error.to_string(),
            "sample schedule exhausted its logical-time range"
        );
        assert!(schedule.is_due_at(u64::MAX).is_err());
        schedule.reset();
        assert!(schedule.is_due_at(0).unwrap());
    }

    #[test]
    fn non_positive_non_finite_and_too_fast_rates_are_rejected() {
        for publish_rate_hz in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let error = SampleSchedule::new("camera", 100.0, publish_rate_hz)
                .expect_err("invalid publish rate must be rejected")
                .to_string();
            assert_eq!(
                error,
                "capability 'camera' publish_rate_hz must be finite and > 0"
            );
        }

        let error = SampleSchedule::new("camera", 100.0, 100.1)
            .expect_err("a producer cannot publish faster than its source")
            .to_string();
        assert_eq!(
            error,
            "capability 'camera' publish_rate_hz (100.1) must not exceed source step_hz (100)"
        );
    }

    #[test]
    fn invalid_source_rate_is_rejected() {
        let error = SampleSchedule::new("camera", f64::NAN, 10.0)
            .expect_err("invalid source rate must be rejected")
            .to_string();
        assert_eq!(error, "capability 'camera' step_hz must be finite and > 0");
    }

    #[test]
    fn an_effective_source_period_slower_than_the_request_sets_the_schedule() {
        let schedule = SampleSchedule::from_source_period_ns("camera", 40_000_000, 30.0).unwrap();
        assert_eq!(schedule.period_ns(), 40_000_000);
    }
}
