//! One module per component-capability family.
//!
//! Each module holds that family's spec, read from the robot's component
//! catalog, and the `NativeXxx` Webots device wrapper that reads or drives it.
//! Sensor families implement [`SimulatedSensor`], which owns the single rule
//! they share: a device is read only on the steps its publish cadence is due
//! on, and a step that produced no observation says so rather than repeating
//! the last one.

pub(crate) mod accelerometer;
pub(crate) mod battery;
pub(crate) mod camera;
pub(crate) mod depth;
pub(crate) mod encoder;
pub(crate) mod gnss;
pub(crate) mod gyroscope;
pub(crate) mod imu;
pub(crate) mod lidar;
pub(crate) mod magnetometer;
pub(crate) mod microphone;
pub(crate) mod mmwave;
pub(crate) mod motor;
pub(crate) mod range;

use anyhow::{Result, bail};
use phoxal::SampleSchedule;
use phoxal::model::identity::CapabilityRef;
use phoxal::model::simulation;

/// The world instant one completed step advanced to.
///
/// Every sensor uses the logical time for its deadline schedule. The encoder
/// and the battery also use the instant to differentiate their reading over
/// the elapsed window to report a rate.
#[derive(Clone, Copy, Debug)]
pub(crate) struct SensorStep {
    pub(crate) time_ns: u64,
}

/// A Webots device the controller reads once per world step.
///
/// The cadence rule lives here rather than in each device: a capability
/// declares a publish rate, the controller resolves it to a logical-time
/// deadline schedule once at setup, and every family applies it the same way.
pub(crate) trait SimulatedSensor {
    /// The endpoint this device produces. A payload *is* its endpoint, so this
    /// one associated type names both the body read from the device and the
    /// contract it is published on; which side of that contract this process
    /// takes is decided where the handle is built.
    type Sample: phoxal::bus::Endpoint;

    /// The mutable publish cadence this capability was bound at.
    fn schedule(&mut self) -> &mut SampleSchedule;

    /// Read the device for `step`.
    ///
    /// `Ok(None)` means the device made no observation in this window. It is
    /// never a fabricated one: a sensor that saw nothing publishes nothing.
    fn read(&mut self, step: SensorStep) -> Result<Option<Self::Sample>>;

    /// Re-anchor this device after Webots starts a new logical world history.
    fn reset(&mut self, logical_time_ns: u64) -> Result<()> {
        let delay_ns = self.schedule().period_ns();
        self.schedule().reanchor_after(logical_time_ns, delay_ns)
    }

    /// The reading for `step`, or `None` when `step` is not a publish deadline.
    fn read_if_due(&mut self, step: SensorStep) -> Result<Option<Self::Sample>> {
        if !self.schedule().is_due_at(step.time_ns)? {
            return Ok(None);
        }
        self.read(step)
    }
}

/// A capability the world samples at a configured rate and the controller
/// publishes at the slower of the requested cadence and the effective device
/// refresh cadence.
#[derive(Clone, Debug)]
pub(crate) struct SampledSpec {
    pub(crate) reference: CapabilityRef,
    pub(crate) schedule: SampleSchedule,
    /// The refresh period the Webots device is enabled with, in milliseconds.
    /// A world may model a device that samples faster than the component
    /// publishes, so this is resolved separately from the publish cadence.
    pub(crate) sampling_period_ms: i32,
}

impl SampledSpec {
    /// Resolve one sampled capability's publish cadence and device refresh
    /// period.
    ///
    /// `simulated` is the world's entry for the same capability, when the
    /// component type authored one. It decides how fast the device samples;
    /// with no entry the device samples at the rate the component publishes.
    ///
    /// # Errors
    ///
    /// Returns an error when the declared publish or sampling rate is not a
    /// positive finite number, which describes no cadence at all.
    pub(crate) fn new(
        reference: CapabilityRef,
        basic_time_step_ms: i32,
        publish_rate_hz: f64,
        simulated: Option<&simulation::Capability>,
    ) -> Result<Self> {
        if basic_time_step_ms <= 0 {
            bail!("Webots basicTimeStep must be > 0");
        }
        let sampling_hz = simulated
            .and_then(Self::simulated_sampling_hz)
            .unwrap_or(publish_rate_hz);
        let requested_sampling_period_ms = Self::sampling_period_ms(sampling_hz)?;
        let sampling_period_ms =
            Self::quantized_sampling_period_ms(requested_sampling_period_ms, basic_time_step_ms)?;
        let effective_sampling_period_ns = u64::try_from(sampling_period_ms)
            .map_err(|_| anyhow::anyhow!("Webots sampling period must be positive"))?
            .checked_mul(1_000_000)
            .ok_or_else(|| anyhow::anyhow!("Webots sampling period overflows nanoseconds"))?;
        let schedule = SampleSchedule::from_source_period_ns(
            &reference.to_string(),
            effective_sampling_period_ns,
            publish_rate_hz,
        )?;
        let mut schedule = schedule;
        schedule.reanchor_after(0, schedule.period_ns())?;
        Ok(Self {
            reference,
            schedule,
            sampling_period_ms,
        })
    }

    /// The rate the world samples this capability at, when it models one.
    ///
    /// The actuator and event families carry no rate: nothing about a motor,
    /// speaker, battery, LED or emergency stop is sampled on a schedule the
    /// world sets.
    fn simulated_sampling_hz(capability: &simulation::Capability) -> Option<f64> {
        match capability {
            simulation::Capability::Encoder(config) => Some(config.sampling_period_hz),
            simulation::Capability::Accelerometer(config) => Some(config.sampling_period_hz),
            simulation::Capability::Gyroscope(config) => Some(config.sampling_period_hz),
            simulation::Capability::Magnetometer(config) => Some(config.sampling_period_hz),
            simulation::Capability::Imu(config) => Some(config.sampling_period_hz),
            simulation::Capability::Gnss(config) => Some(config.sampling_period_hz),
            simulation::Capability::Camera(config) => Some(config.sampling_period_hz),
            simulation::Capability::Depth(config) => Some(config.sampling_period_hz),
            simulation::Capability::Range(config) => Some(config.sampling_period_hz),
            simulation::Capability::Lidar(config) => Some(config.sampling_period_hz),
            simulation::Capability::Mmwave(config) => Some(config.sampling_period_hz),
            simulation::Capability::Microphone(config) => Some(config.sampling_period_hz),
            simulation::Capability::Motor(_)
            | simulation::Capability::EmergencyStop
            | simulation::Capability::Speaker
            | simulation::Capability::Battery
            | simulation::Capability::Led => None,
        }
    }

    /// The effective refresh period Webots samples the device at. It is a
    /// positive whole-millisecond period rounded up to the world's
    /// `basicTimeStep` grid before the device is enabled.
    fn sampling_period_ms(rate_hz: f64) -> Result<i32> {
        if !rate_hz.is_finite() || rate_hz <= 0.0 {
            bail!("sampling_period_hz must be finite and > 0");
        }
        let period_ms = (1000.0 / rate_hz).round().max(1.0);
        if period_ms > i32::MAX as f64 {
            bail!("sampling period does not fit in Webots milliseconds");
        }
        Ok(period_ms as i32)
    }

    /// Webots updates a device on the world's integer-step grid. Rounding a
    /// requested period upward to that grid is conservative: the schedule can
    /// never publish a repeated value before the device has had a chance to
    /// refresh it.
    fn quantized_sampling_period_ms(requested_ms: i32, basic_time_step_ms: i32) -> Result<i32> {
        if basic_time_step_ms <= 0 {
            bail!("Webots basicTimeStep must be > 0");
        }
        let requested = u64::try_from(requested_ms)
            .map_err(|_| anyhow::anyhow!("Webots sampling period must be positive"))?;
        let basic = u64::try_from(basic_time_step_ms)
            .map_err(|_| anyhow::anyhow!("Webots basicTimeStep must be positive"))?;
        let steps = requested
            .checked_add(basic - 1)
            .and_then(|value| value.checked_div(basic))
            .ok_or_else(|| anyhow::anyhow!("Webots sampling period quantization overflowed"))?;
        let quantized = steps
            .checked_mul(basic)
            .ok_or_else(|| anyhow::anyhow!("Webots sampling period quantization overflowed"))?;
        i32::try_from(quantized)
            .map_err(|_| anyhow::anyhow!("Webots sampling period does not fit in milliseconds"))
    }
}

/// Declare a three-axis Webots sensor that reports one `[f32; 3]` field.
///
/// Accelerometer, gyroscope and magnetometer are the same device wrapper three
/// times over: open the named device, enable it at the sampled period, and map
/// its three doubles into one contract field. Only the Webots accessor, the
/// device type and the body field differ, so they are named here rather than
/// copied.
macro_rules! vector_sensor {
    (
        $native:ident,
        $device:ty,
        $accessor:ident,
        $sample:ty,
        $field:ident $(,)?
    ) => {
        pub(crate) struct $native {
            device: $device,
            spec: $crate::capabilities::SampledSpec,
        }

        impl $native {
            pub(crate) fn new(
                webots: &webots_rs::Webots,
                spec: &$crate::capabilities::SampledSpec,
            ) -> ::anyhow::Result<Self> {
                let device = webots.$accessor(spec.reference.to_string())?;
                device.enable(spec.sampling_period_ms)?;
                Ok(Self {
                    device,
                    spec: spec.clone(),
                })
            }
        }

        impl $crate::capabilities::SimulatedSensor for $native {
            type Sample = $sample;

            fn schedule(&mut self) -> &mut phoxal::SampleSchedule {
                &mut self.spec.schedule
            }

            fn read(
                &mut self,
                _step: $crate::capabilities::SensorStep,
            ) -> ::anyhow::Result<Option<Self::Sample>> {
                // A captured type cannot be written directly in struct-literal
                // position, so the body type is named through a local alias.
                type Sample = $sample;
                Sample::try_new(self.device.values()?.map(|value| value as f32))
                    .map(Some)
                    .map_err(::anyhow::Error::from)
            }
        }
    };
}

pub(crate) use vector_sensor;

#[cfg(test)]
mod tests {
    use super::SampledSpec;
    use phoxal::model::identity::CapabilityRef;
    use phoxal::model::simulation;

    #[test]
    fn webots_sampling_period_is_quantized_up_to_the_world_grid() {
        assert_eq!(SampledSpec::sampling_period_ms(30.0).unwrap(), 33);
        assert_eq!(
            SampledSpec::quantized_sampling_period_ms(33, 10).unwrap(),
            40
        );
        assert_eq!(
            SampledSpec::quantized_sampling_period_ms(50, 10).unwrap(),
            50
        );
    }

    #[test]
    fn quantized_device_refresh_sets_the_effective_publish_cadence() {
        let reference: CapabilityRef = "front_camera.rgb".parse().unwrap();
        let spec = SampledSpec::new(reference, 10, 30.0, None).unwrap();
        assert_eq!(spec.sampling_period_ms, 40);
        assert_eq!(spec.schedule.period_ns(), 40_000_000);

        let mut schedule = spec.schedule;
        assert!(!schedule.is_due_at(0).unwrap());
        assert!(!schedule.is_due_at(39_999_999).unwrap());
        assert!(schedule.is_due_at(40_000_000).unwrap());
    }

    #[test]
    fn a_faster_device_preserves_the_requested_publish_cadence() {
        let reference: CapabilityRef = "front_camera.rgb".parse().unwrap();
        let simulated = simulation::Capability::Camera(simulation::Camera {
            sampling_period_hz: 100.0,
            ..simulation::Camera::default()
        });
        let spec = SampledSpec::new(reference, 10, 30.0, Some(&simulated)).unwrap();
        assert_eq!(spec.sampling_period_ms, 10);
        assert_eq!(spec.schedule.period_ns(), 33_333_333);
    }

    #[test]
    fn the_first_deadline_waits_for_the_device_refresh() {
        let reference: CapabilityRef = "front_camera.rgb".parse().unwrap();
        let spec = SampledSpec::new(reference, 10, 20.0, None).unwrap();
        let mut schedule = spec.schedule;
        assert!(!schedule.is_due_at(0).unwrap());
        assert!(schedule.is_due_at(50_000_000).unwrap());
    }
}
