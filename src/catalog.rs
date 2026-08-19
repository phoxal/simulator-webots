//! What the robot's component catalog asks this controller to simulate.
//!
//! The catalog is read once at startup and fixes the binding order for
//! everything downstream: the devices the backend opens and the bus handles
//! the controller publishes on are built from this one sequence, so a
//! capability's device and its handle are never matched up by position again.

use std::collections::BTreeMap;

use anyhow::{Result, anyhow, bail};
use phoxal::model::Robot;
use phoxal::model::component::capability::{Capability, CapabilityKind};
use phoxal::model::identity::CapabilityRef;

use crate::capabilities::SampledSpec;
use crate::capabilities::battery::BatterySpec;
use crate::capabilities::camera::CameraSpec;
use crate::capabilities::depth::DepthSpec;
use crate::capabilities::encoder::EncoderSpec;
use crate::capabilities::lidar::LidarSpec;
use crate::capabilities::motor::MotorSpec;
use crate::capabilities::range::RangeSpec;

/// One capability this controller simulates, carrying everything binding it
/// needs.
///
/// Each family keeps its own spec type: what a lidar needs to bind has nothing
/// in common with what a battery needs, and collapsing them would only move
/// the difference into a runtime check.
pub(crate) enum CapabilitySpec {
    Motor(MotorSpec),
    Encoder(EncoderSpec),
    Imu(SampledSpec),
    Accelerometer(SampledSpec),
    Gyroscope(SampledSpec),
    Range(RangeSpec),
    Camera(CameraSpec),
    Depth(DepthSpec),
    Gnss(SampledSpec),
    Magnetometer(SampledSpec),
    Lidar(LidarSpec),
    Mmwave(SampledSpec),
    Microphone(SampledSpec),
    Battery(BatterySpec),
}

impl CapabilitySpec {
    /// The capability this spec binds.
    pub(crate) fn reference(&self) -> &CapabilityRef {
        match self {
            Self::Motor(spec) => &spec.reference,
            Self::Encoder(spec) => &spec.sampled.reference,
            Self::Imu(spec)
            | Self::Accelerometer(spec)
            | Self::Gyroscope(spec)
            | Self::Gnss(spec)
            | Self::Magnetometer(spec)
            | Self::Mmwave(spec)
            | Self::Microphone(spec) => &spec.reference,
            Self::Range(spec) => &spec.sampled.reference,
            Self::Camera(spec) => &spec.sampled.reference,
            Self::Depth(spec) => &spec.sampled.reference,
            Self::Lidar(spec) => &spec.sampled.reference,
            Self::Battery(spec) => &spec.sampled.reference,
        }
    }

    /// The device kind this spec binds.
    pub(crate) const fn kind(&self) -> CapabilityKind {
        match self {
            Self::Motor(_) => CapabilityKind::Motor,
            Self::Encoder(_) => CapabilityKind::Encoder,
            Self::Imu(_) => CapabilityKind::Imu,
            Self::Accelerometer(_) => CapabilityKind::Accelerometer,
            Self::Gyroscope(_) => CapabilityKind::Gyroscope,
            Self::Range(_) => CapabilityKind::Range,
            Self::Camera(_) => CapabilityKind::Camera,
            Self::Depth(_) => CapabilityKind::Depth,
            Self::Gnss(_) => CapabilityKind::Gnss,
            Self::Magnetometer(_) => CapabilityKind::Magnetometer,
            Self::Lidar(_) => CapabilityKind::Lidar,
            Self::Mmwave(_) => CapabilityKind::Mmwave,
            Self::Microphone(_) => CapabilityKind::Microphone,
            Self::Battery(_) => CapabilityKind::Battery,
        }
    }
}

/// Every capability of this robot's Webots model the controller simulates, in
/// the robot's canonical `(component id, capability id)` order.
pub(crate) struct CapabilityCatalog {
    specs: Vec<CapabilitySpec>,
}

impl CapabilityCatalog {
    /// Read the manifest's component catalog into binding specs.
    ///
    /// How a world models a component is a fact of the component *type*, so it
    /// is read through [`Robot::component`]'s joined view rather than by
    /// looking up an instance in a second map: the manifest carries one
    /// `component_types` entry per type, simulation included, and the view is
    /// what keeps this from being a hand-written join.
    ///
    /// # Errors
    ///
    /// Returns an error when a declared capability describes an invalid or
    /// source-too-fast cadence, when the robot declares more batteries than a
    /// Webots robot can hold, or when it requests an effect with no defined
    /// source authority.
    pub(crate) fn from_robot(robot: &Robot, basic_time_step_ms: i32) -> Result<Self> {
        let mut specs = Vec::new();

        for reference in robot.capability_refs(|_| true) {
            let capability = robot.capability(&reference).ok_or_else(|| {
                anyhow!("robot declares capability {reference} but does not resolve it")
            })?;
            let simulated = robot
                .component(reference.component_id.as_str())
                .and_then(|component| component.simulation())
                .and_then(|simulation| simulation.capability(reference.capability_id.as_str()));

            match capability {
                Capability::Motor(config) => specs.push(CapabilitySpec::Motor(MotorSpec {
                    reference,
                    actuator_type: config.command,
                    gear_ratio: config.gear_ratio,
                })),
                Capability::Encoder(config) => specs.push(CapabilitySpec::Encoder(EncoderSpec {
                    sampled: SampledSpec::new(
                        reference,
                        basic_time_step_ms,
                        config.publish_rate_hz,
                        simulated,
                    )?,
                    gear_ratio: config.gear_ratio,
                })),
                Capability::Imu(config) => specs.push(CapabilitySpec::Imu(SampledSpec::new(
                    reference,
                    basic_time_step_ms,
                    config.publish_rate_hz,
                    simulated,
                )?)),
                Capability::Accelerometer(config) => {
                    specs.push(CapabilitySpec::Accelerometer(SampledSpec::new(
                        reference,
                        basic_time_step_ms,
                        config.publish_rate_hz,
                        simulated,
                    )?));
                }
                Capability::Gyroscope(config) => {
                    specs.push(CapabilitySpec::Gyroscope(SampledSpec::new(
                        reference,
                        basic_time_step_ms,
                        config.publish_rate_hz,
                        simulated,
                    )?));
                }
                Capability::Range(config) => specs.push(CapabilitySpec::Range(RangeSpec {
                    sampled: SampledSpec::new(
                        reference,
                        basic_time_step_ms,
                        config.publish_rate_hz,
                        simulated,
                    )?,
                    min_range_m: config.min_range_m as f32,
                    max_range_m: config.max_range_m as f32,
                })),
                Capability::Camera(config) => specs.push(CapabilitySpec::Camera(CameraSpec {
                    sampled: SampledSpec::new(
                        reference,
                        basic_time_step_ms,
                        config.publish_rate_hz,
                        simulated,
                    )?,
                    mode: config.mode,
                    width: config.width_px,
                    height: config.height_px,
                })),
                Capability::Depth(config) => specs.push(CapabilitySpec::Depth(DepthSpec {
                    sampled: SampledSpec::new(
                        reference,
                        basic_time_step_ms,
                        config.publish_rate_hz,
                        simulated,
                    )?,
                    width: config.width_px,
                    height: config.height_px,
                })),
                Capability::Gnss(config) => specs.push(CapabilitySpec::Gnss(SampledSpec::new(
                    reference,
                    basic_time_step_ms,
                    config.publish_rate_hz,
                    simulated,
                )?)),
                Capability::Magnetometer(config) => {
                    specs.push(CapabilitySpec::Magnetometer(SampledSpec::new(
                        reference,
                        basic_time_step_ms,
                        config.publish_rate_hz,
                        simulated,
                    )?));
                }
                Capability::Lidar(config) => specs.push(CapabilitySpec::Lidar(LidarSpec {
                    sampled: SampledSpec::new(
                        reference,
                        basic_time_step_ms,
                        config.publish_rate_hz,
                        simulated,
                    )?,
                    output: config.output,
                })),
                Capability::Mmwave(config) => {
                    specs.push(CapabilitySpec::Mmwave(SampledSpec::new(
                        reference,
                        basic_time_step_ms,
                        config.publish_rate_hz,
                        simulated,
                    )?));
                }
                Capability::Microphone(config) => {
                    specs.push(CapabilitySpec::Microphone(SampledSpec::new(
                        reference,
                        basic_time_step_ms,
                        config.publish_rate_hz,
                        simulated,
                    )?));
                }
                Capability::Battery(config) => specs.push(CapabilitySpec::Battery(BatterySpec {
                    sampled: SampledSpec::new(
                        reference,
                        basic_time_step_ms,
                        config.publish_rate_hz,
                        simulated,
                    )?,
                    voltage_v: config.voltage_v,
                    capacity_ah: config.capacity_ah,
                })),
                // Unlike a motor, no participant owns LED or speaker effects
                // in the current graph. Applying either command from every
                // source would turn arrival order into authority. Reject the
                // simulated graph at startup rather than silently choosing an
                // owner or applying an unauthorised effect.
                Capability::Led(_) | Capability::Speaker(_) => bail!(
                    "Webots {} effects are unsupported before v1: capability {reference} \
                     has no declared command authority",
                    capability.kind()
                ),
                // Webots has no button, switch, or toggle node, so nothing in
                // a simulated world can engage or release an e-stop. Leaving
                // it unpublished is the honest state: `motion` fails closed on
                // a component it never hears from.
                Capability::EmergencyStop(_) => {
                    tracing::debug!(
                        target: crate::LOG_TARGET,
                        capability = %reference,
                        kind = %capability.kind(),
                        "Webots models no emergency-stop control, so this capability is \
                         not simulated"
                    );
                }
            }
        }

        let catalog = Self { specs };
        catalog.validate()?;
        Ok(catalog)
    }

    /// The specs to bind, in binding order.
    pub(crate) fn specs(&self) -> &[CapabilitySpec] {
        &self.specs
    }

    /// How many capabilities of each kind this robot declares, for the
    /// controller's readiness log.
    pub(crate) fn kind_counts(&self) -> BTreeMap<&'static str, usize> {
        let mut counts = BTreeMap::new();
        for spec in &self.specs {
            *counts.entry(spec.kind().as_str()).or_default() += 1;
        }
        counts
    }

    /// Webots gives a robot exactly one battery sensor, so a second battery
    /// capability would report the first one's energy under another name.
    fn validate(&self) -> Result<()> {
        let batteries = self
            .specs
            .iter()
            .filter(|spec| matches!(spec, CapabilitySpec::Battery(_)))
            .map(|spec| spec.reference().to_string())
            .collect::<Vec<_>>();
        if batteries.len() > 1 {
            bail!(
                "Webots models one battery per robot, but this robot declares {}: {}. \
                 Keep exactly one battery capability for a simulated robot.",
                batteries.len(),
                batteries.join(", ")
            );
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use phoxal::model::builder::RobotBuilder;
    use phoxal::model::simulation;

    use super::CapabilityCatalog;

    /// The canonical model shape emitted by a two-wheel rover: two encoder
    /// instances of one type, authored at 50 Hz, with the simulator device at
    /// the 62.5 Hz cadence a 16 ms world grid gives. This runs the exact
    /// catalog seam controller startup uses, so a component/simulation mismatch
    /// cannot survive until an external Webots process is launched.
    #[test]
    fn a_two_wheel_rover_catalog_resolves_at_a_sixteen_millisecond_world_step() {
        let robot = RobotBuilder::new("hello-rover")
            .component_type("wheel_drive", |wheel_drive| {
                wheel_drive.encoder("encoder", "motor_joint").simulated(
                    "encoder",
                    simulation::Capability::Encoder(simulation::Encoder {
                        sampling_period_hz: 62.5,
                        ..Default::default()
                    }),
                )
            })
            .component("left_drive", "wheel_drive")
            .component("right_drive", "wheel_drive")
            .build()
            .expect("hello-rover catalog model must build");

        let catalog = CapabilityCatalog::from_robot(&robot, 16)
            .expect("hello-rover capabilities must be schedulable at 16 ms");
        assert_eq!(catalog.specs().len(), 2);
    }

    #[test]
    fn effect_capabilities_fail_closed_without_a_declared_authority() {
        let led = RobotBuilder::new("led-rover")
            .component_type("status_panel", |panel| panel.led("status", "base_link"))
            .component("status_panel", "status_panel")
            .build()
            .expect("LED model is otherwise valid");
        let led_error = match CapabilityCatalog::from_robot(&led, 16) {
            Ok(_) => panic!("an LED effect must not pick an implicit source"),
            Err(error) => error,
        };
        assert!(
            led_error
                .to_string()
                .contains("Webots led effects are unsupported before v1")
        );

        let speaker = RobotBuilder::new("speaker-rover")
            .component_type("speaker_panel", |panel| panel.speaker("alert", "base_link"))
            .component("speaker_panel", "speaker_panel")
            .build()
            .expect("speaker model is otherwise valid");
        let speaker_error = match CapabilityCatalog::from_robot(&speaker, 16) {
            Ok(_) => panic!("a speaker effect must not pick an implicit source"),
            Err(error) => error,
        };
        assert!(
            speaker_error
                .to_string()
                .contains("Webots speaker effects are unsupported before v1")
        );
    }
}
