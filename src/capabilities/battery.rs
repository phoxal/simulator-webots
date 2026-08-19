//! Battery capability: publishes `component::battery::State` from the Webots
//! robot battery sensor.
//!
//! The battery is the one capability Webots hangs off the robot rather than off
//! a device: `wb_robot_battery_sensor_get_value` returns "the present energy
//! level of the battery expressed in Joules", and `-1.0` when the world's
//! `Robot` node left its `battery` field empty. There is exactly one per robot,
//! so exactly one declared battery capability may bind it.
//!
//! Joules alone do not make a battery state. The pack's nominal voltage and
//! capacity come from the component manifest and turn that energy into the
//! reported charge ratio and current. Voltage is reported as the declared
//! nominal: Webots models pack energy, not terminal voltage, and a discharge
//! curve invented here would describe no simulated hardware.

use anyhow::Result;
use phoxal::api;

use super::{SampledSpec, SensorStep, SimulatedSensor};

const SECONDS_PER_HOUR: f64 = 3_600.0;

#[derive(Clone, Debug)]
pub(crate) struct BatterySpec {
    pub(crate) sampled: SampledSpec,
    pub(crate) voltage_v: f64,
    pub(crate) capacity_ah: f64,
}

impl BatterySpec {
    /// The energy a full pack holds, in Joules - the denominator Webots cannot
    /// supply, since reading the `Robot` node's `battery` field would take a
    /// supervisor this controller never opens.
    fn full_energy_j(&self) -> f64 {
        self.voltage_v * self.capacity_ah * SECONDS_PER_HOUR
    }

    /// The pack state `energy_j` describes, given the previous reading.
    fn state(
        &self,
        energy_j: f64,
        previous: Option<(f64, u64)>,
        time_ns: u64,
    ) -> Result<api::component::battery::State> {
        let charge_ratio = (energy_j / self.full_energy_j()).clamp(0.0, 1.0);
        // Current is what the pack is actually delivering: the energy it lost
        // over the elapsed window, at the pack's nominal voltage. Positive is
        // discharge.
        let current_a = previous
            .and_then(|(previous_energy_j, previous_time_ns)| {
                let dt_ns = time_ns.saturating_sub(previous_time_ns);
                if dt_ns == 0 {
                    return None;
                }
                let watts = (previous_energy_j - energy_j) * 1_000_000_000.0 / dt_ns as f64;
                Some(watts / self.voltage_v)
            })
            .unwrap_or(0.0);
        api::component::battery::State::try_new(
            self.voltage_v as f32,
            current_a as f32,
            charge_ratio as f32,
        )
        .map_err(anyhow::Error::from)
    }
}

pub(crate) struct NativeBattery {
    sensor: webots_rs::device::battery_sensor::BatterySensor,
    spec: BatterySpec,
    last: Option<(f64, u64)>,
    reported_absent: bool,
}

impl NativeBattery {
    pub(crate) fn new(spec: &BatterySpec) -> Result<Self> {
        let sensor = webots_rs::device::battery_sensor::BatterySensor::new();
        sensor.enable(spec.sampled.sampling_period_ms)?;
        Ok(Self {
            sensor,
            spec: spec.clone(),
            last: None,
            reported_absent: false,
        })
    }
}

impl SimulatedSensor for NativeBattery {
    type Sample = api::component::battery::State;

    fn schedule(&mut self) -> &mut phoxal::SampleSchedule {
        &mut self.spec.sampled.schedule
    }

    fn reset(&mut self, logical_time_ns: u64) -> Result<()> {
        let delay_ns = self.spec.sampled.schedule.period_ns();
        self.spec
            .sampled
            .schedule
            .reanchor_after(logical_time_ns, delay_ns)?;
        self.last = None;
        Ok(())
    }

    /// Like the encoder, the battery differentiates its reading over the step
    /// window: the pack's current is the energy it lost since the last
    /// published reading.
    fn read(&mut self, step: SensorStep) -> Result<Option<Self::Sample>> {
        let energy_j = self.sensor.get_value()?;
        // A world whose Robot node has no `battery` field models no battery.
        // Saying so once beats either a silent gap or a log line per step.
        if !energy_j.is_finite() || energy_j < 0.0 {
            if !self.reported_absent {
                self.reported_absent = true;
                tracing::warn!(
                    target: crate::LOG_TARGET,
                    capability = %self.spec.sampled.reference,
                    "this world's Robot node declares no `battery` field, so the battery \
                     capability publishes nothing; add [energy, max_energy, recharge] to the \
                     Robot node to simulate it"
                );
            }
            return Ok(None);
        }
        let state = self.spec.state(energy_j, self.last, step.time_ns)?;
        self.last = Some((energy_j, step.time_ns));
        Ok(Some(state))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use phoxal::SampleSchedule;

    fn spec() -> BatterySpec {
        BatterySpec {
            sampled: SampledSpec {
                reference: "pack.battery"
                    .parse()
                    .expect("a valid capability reference"),
                schedule: SampleSchedule::from_source_period_ns("pack.battery", 10_000_000, 100.0)
                    .expect("a valid publish rate"),
                sampling_period_ms: 100,
            },
            voltage_v: 16.0,
            capacity_ah: 10.0,
        }
    }

    #[test]
    fn charge_ratio_is_energy_over_a_full_pack() {
        let spec = spec();
        let full = spec.full_energy_j();
        assert_eq!(spec.state(full, None, 0).unwrap().charge_ratio, 1.0);
        assert_eq!(spec.state(full / 2.0, None, 0).unwrap().charge_ratio, 0.5);
        assert_eq!(spec.state(0.0, None, 0).unwrap().charge_ratio, 0.0);
    }

    /// A world may recharge past the declared capacity, or the author may
    /// declare a smaller pack than the world holds. Neither makes a battery
    /// more than full.
    #[test]
    fn a_pack_over_its_declared_capacity_still_reads_full() {
        let spec = spec();
        let over = spec.full_energy_j() * 2.0;
        assert_eq!(spec.state(over, None, 0).unwrap().charge_ratio, 1.0);
    }

    #[test]
    fn current_is_the_energy_lost_over_the_window_at_nominal_voltage() {
        let spec = spec();
        // 160 J lost in one second at 16 V is 160 W, so 10 A.
        let state = spec
            .state(1_000.0, Some((1_160.0, 0)), 1_000_000_000)
            .unwrap();
        assert!((state.current_a - 10.0).abs() < 1e-3);
        assert_eq!(state.voltage_v, 16.0);
    }

    #[test]
    fn a_charging_pack_reports_negative_current() {
        let spec = spec();
        let state = spec
            .state(1_160.0, Some((1_000.0, 0)), 1_000_000_000)
            .unwrap();
        assert!(state.current_a < 0.0);
    }

    #[test]
    fn the_first_reading_has_no_window_to_differentiate() {
        assert_eq!(spec().state(1_000.0, None, 0).unwrap().current_a, 0.0);
        assert_eq!(
            spec()
                .state(1_000.0, Some((2_000.0, 5)), 5)
                .unwrap()
                .current_a,
            0.0
        );
    }
}
