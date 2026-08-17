//! One capability's Webots device bound to the bus handle that serves it.
//!
//! Binding the device and its handle into one record is what keeps them from
//! being re-paired by position on every step: a sample is published on the
//! handle it was just read through, in the same call, and a queued command
//! already sits next to the device it drives. The controller holds one
//! `Vec<CapabilityChannel>` in the robot's canonical capability order, so there
//! is exactly one sequence to keep straight instead of sixteen parallel ones.

use anyhow::Result;
use phoxal_bus::{
    BusHandle, CaptureStamp, FixedSourceLease, LocalInstant, Observed, ParticipantId,
    ParticipantReadyEvents, SampleContract, SamplePublisher, SetpointReceiver, StatePublisher,
    StepStamp, WorldStepToken,
};
use phoxal_model::identity::CapabilityRef;
use phoxal_protocol::robot as api;
use std::time::Duration;

use crate::capabilities::accelerometer::NativeAccelerometer;
use crate::capabilities::battery::NativeBattery;
use crate::capabilities::camera::NativeCamera;
use crate::capabilities::depth::NativeDepth;
use crate::capabilities::encoder::NativeEncoder;
use crate::capabilities::gnss::NativeGnss;
use crate::capabilities::gyroscope::NativeGyroscope;
use crate::capabilities::imu::NativeImu;
use crate::capabilities::lidar::NativeLidar;
use crate::capabilities::magnetometer::NativeMagnetometer;
use crate::capabilities::microphone::NativeMicrophone;
use crate::capabilities::mmwave::NativeMmwave;
use crate::capabilities::motor::NativeMotor;
use crate::capabilities::range::NativeRange;
use crate::capabilities::{SensorStep, SimulatedSensor};
use crate::catalog::CapabilitySpec;

const MOTOR_SOURCE_SILENCE: Duration = Duration::from_millis(150);

/// The one participant allowed to command a simulated motor.
///
/// The authority policy itself is `phoxal-bus`'s [`FixedSourceLease`], not
/// anything this binary invents: leaving the facade behind changed how the
/// receiver is built, not who may drive the wheels.
const MOTOR_COMMAND_AUTHORITY: &str = "drive";

/// A sensor device and the measurement handle it publishes on.
struct SensorChannel<S: SimulatedSensor>
where
    S::Endpoint: SampleContract<Payload = S::Sample>,
{
    device: S,
    publisher: SamplePublisher<S::Endpoint>,
}

impl<S: SimulatedSensor> SensorChannel<S>
where
    S::Endpoint: SampleContract<Payload = S::Sample>,
{
    fn reset(&mut self, logical_time_ns: u64) -> Result<()> {
        self.device.reset(logical_time_ns)
    }

    /// Read this step's sample, if there is one, and publish it on this
    /// channel's own handle.
    ///
    /// The read and the publish are one act, on the one handle the sample
    /// belongs to, so no body ever exists apart from the publisher that serves
    /// it and no step allocates a queue to hold it.
    fn publish_due(&mut self, step: SensorStep, captured_at: CaptureStamp) -> Result<()> {
        if let Some(sample) = self.device.read_if_due(step)? {
            self.publisher.publish(captured_at, sample)?;
        }
        Ok(())
    }
}

/// The motor device, the setpoints the graph sent it, and the authority that
/// decides which of them may be applied.
///
/// The motor is the only device the graph drives: LED and speaker effects are
/// refused at startup for want of a declared command authority, and Webots
/// models no emergency-stop control at all. One concrete channel therefore says
/// exactly what happens on a step, instead of a family-generic actuator with a
/// single implementation.
struct MotorChannel {
    device: NativeMotor,
    commands: SetpointReceiver<api::endpoint::component::motor::CommandEndpoint>,
    authority: FixedSourceLease<api::component::motor::Command>,
    ready: ParticipantReadyEvents,
}

impl MotorChannel {
    /// Every queued setpoint, in receiver order, carrying the trusted transport
    /// provenance fixed-source admission needs.
    ///
    /// Draining cannot fail: `try_recv` either yields a value or reports the
    /// queue empty.
    fn drain_commands(&self) -> Vec<Observed<api::component::motor::Command>> {
        let mut pending = Vec::new();
        while let Some(observed) = self.commands.try_recv() {
            pending.push(observed);
        }
        pending
    }

    fn apply_backlog(&mut self) -> Result<()> {
        while let Some(event) = self.ready.try_recv() {
            self.authority.update_ready_event(&event);
        }
        if self.ready.overflowed() {
            self.authority.mark_ready_overflow();
        }
        let Some(host_now) = LocalInstant::try_now() else {
            // A receiver that cannot stamp its own clock cannot prove that a
            // retained motor command is still live.  Drain and park before
            // the next Webots step; the step loop surfaces the latched clock
            // fault as a controller failure.
            drop(self.drain_commands());
            self.authority.clear();
            return self.device.stop();
        };
        let pending = self.drain_commands();
        admit_pending(&mut self.authority, pending);
        match self.authority.live_host(host_now) {
            Some(command) => self.device.apply(command),
            None => self.device.stop(),
        }
    }

    fn park(&mut self) -> Result<()> {
        // Whatever the graph sent that the stopped loop will never apply goes
        // with it; leaving it queued would apply it to a later world.
        drop(self.drain_commands());
        self.device.stop()
    }
}

/// Admit every pending command before selecting the held value to apply.
///
/// The receiver preserves one pending command per producer.  Offering that
/// complete set to the lease keeps an unauthorised producer from coalescing
/// away Drive's candidate before the authority check can reject it.
fn admit_pending<B>(authority: &mut FixedSourceLease<B>, pending: Vec<Observed<B>>) {
    for observed in pending {
        let _decision = authority.offer(
            observed.metadata.source.participant_source(),
            observed.metadata.sequence,
            observed.observed_at,
            observed.body,
        );
    }
}

/// The battery device and the state handle it publishes on.
///
/// The battery is bound on its own: it is the one capability Webots hangs off
/// the robot rather than off a named device, and what it reports is state
/// rather than a measurement.
struct BatteryChannel {
    device: NativeBattery,
    publisher: StatePublisher<api::endpoint::component::battery::StateEndpoint>,
}

/// One bound capability: the reference it was declared under, and the device
/// and handle serving it.
pub(crate) struct CapabilityChannel {
    reference: CapabilityRef,
    binding: CapabilityBinding,
}

/// The device and handle behind one capability, statically typed by the family
/// it belongs to.
enum CapabilityBinding {
    Motor(MotorChannel),
    Encoder(SensorChannel<NativeEncoder>),
    Imu(SensorChannel<NativeImu>),
    Accelerometer(SensorChannel<NativeAccelerometer>),
    Gyroscope(SensorChannel<NativeGyroscope>),
    Range(SensorChannel<NativeRange>),
    Camera(SensorChannel<NativeCamera>),
    Depth(SensorChannel<NativeDepth>),
    Gnss(SensorChannel<NativeGnss>),
    Magnetometer(SensorChannel<NativeMagnetometer>),
    Lidar(SensorChannel<NativeLidar>),
    Mmwave(SensorChannel<NativeMmwave>),
    Microphone(SensorChannel<NativeMicrophone>),
    Battery(BatteryChannel),
}

impl CapabilityChannel {
    /// Open this capability's Webots device and attach the bus handle it is
    /// served on.
    ///
    /// The handles are built straight from the session rather than through a
    /// participant setup context: this process is an ordinary bus client, so
    /// there is no runner holding a context for it and nothing between the
    /// endpoint's topic builder and the handle that publishes on it.
    pub(crate) async fn bind(
        bus: &BusHandle,
        webots: &webots_rs::Webots,
        spec: &CapabilitySpec,
    ) -> Result<Self> {
        let reference = spec.reference().clone();
        // Every topic under this capability starts from the same component
        // segment; the leaf method is what names the contract.
        let component = || api::topic::owner().component(&reference.component_id);
        let id = &reference.capability_id;
        let binding = match spec {
            CapabilitySpec::Motor(spec) => {
                let drive = ParticipantId::new(MOTOR_COMMAND_AUTHORITY)?;
                CapabilityBinding::Motor(MotorChannel {
                    device: NativeMotor::new(webots, spec)?,
                    commands: SetpointReceiver::new(bus, &component()?.motor(id)?.command())
                        .await?,
                    authority: FixedSourceLease::new(
                        "component/motor/command",
                        drive.clone(),
                        MOTOR_SOURCE_SILENCE,
                        Duration::MAX,
                    ),
                    ready: bus.participant_ready_events_for(&drive).await?,
                })
            }
            CapabilitySpec::Encoder(spec) => CapabilityBinding::Encoder(SensorChannel {
                device: NativeEncoder::new(webots, spec)?,
                publisher: SamplePublisher::new(bus.clone(), &component()?.encoder(id)?.sample())?,
            }),
            CapabilitySpec::Imu(spec) => CapabilityBinding::Imu(SensorChannel {
                device: NativeImu::new(webots, spec)?,
                publisher: SamplePublisher::new(bus.clone(), &component()?.imu(id)?.sample())?,
            }),
            CapabilitySpec::Accelerometer(spec) => {
                CapabilityBinding::Accelerometer(SensorChannel {
                    device: NativeAccelerometer::new(webots, spec)?,
                    publisher: SamplePublisher::new(
                        bus.clone(),
                        &component()?.accelerometer(id)?.sample(),
                    )?,
                })
            }
            CapabilitySpec::Gyroscope(spec) => CapabilityBinding::Gyroscope(SensorChannel {
                device: NativeGyroscope::new(webots, spec)?,
                publisher: SamplePublisher::new(
                    bus.clone(),
                    &component()?.gyroscope(id)?.sample(),
                )?,
            }),
            CapabilitySpec::Range(spec) => CapabilityBinding::Range(SensorChannel {
                device: NativeRange::new(webots, spec)?,
                publisher: SamplePublisher::new(bus.clone(), &component()?.range(id)?.sample())?,
            }),
            CapabilitySpec::Camera(spec) => CapabilityBinding::Camera(SensorChannel {
                device: NativeCamera::new(webots, spec)?,
                publisher: SamplePublisher::new(bus.clone(), &component()?.camera(id)?.frame())?,
            }),
            CapabilitySpec::Depth(spec) => CapabilityBinding::Depth(SensorChannel {
                device: NativeDepth::new(webots, spec)?,
                publisher: SamplePublisher::new(bus.clone(), &component()?.depth(id)?.frame())?,
            }),
            CapabilitySpec::Gnss(spec) => CapabilityBinding::Gnss(SensorChannel {
                device: NativeGnss::new(webots, spec)?,
                publisher: SamplePublisher::new(bus.clone(), &component()?.gnss(id)?.sample())?,
            }),
            CapabilitySpec::Magnetometer(spec) => CapabilityBinding::Magnetometer(SensorChannel {
                device: NativeMagnetometer::new(webots, spec)?,
                publisher: SamplePublisher::new(
                    bus.clone(),
                    &component()?.magnetometer(id)?.sample(),
                )?,
            }),
            CapabilitySpec::Lidar(spec) => CapabilityBinding::Lidar(SensorChannel {
                device: NativeLidar::new(webots, spec)?,
                publisher: SamplePublisher::new(bus.clone(), &component()?.lidar(id)?.scan())?,
            }),
            CapabilitySpec::Mmwave(spec) => CapabilityBinding::Mmwave(SensorChannel {
                device: NativeMmwave::new(webots, spec)?,
                publisher: SamplePublisher::new(bus.clone(), &component()?.mmwave(id)?.scan())?,
            }),
            CapabilitySpec::Microphone(spec) => CapabilityBinding::Microphone(SensorChannel {
                device: NativeMicrophone::new(webots, spec)?,
                publisher: SamplePublisher::new(
                    bus.clone(),
                    &component()?.microphone(id)?.frame(),
                )?,
            }),
            CapabilitySpec::Battery(spec) => CapabilityBinding::Battery(BatteryChannel {
                device: NativeBattery::new(spec)?,
                publisher: StatePublisher::new(bus.clone(), &component()?.battery(id)?.state())?,
            }),
        };
        Ok(Self { reference, binding })
    }

    /// The capability this channel serves.
    pub(crate) const fn reference(&self) -> &CapabilityRef {
        &self.reference
    }

    /// Apply everything the graph asked this capability to do since the
    /// previous step. Sensors have nothing to apply: they are read after the
    /// world advances, not before.
    pub(crate) fn apply_backlog(&mut self) -> Result<()> {
        match &mut self.binding {
            CapabilityBinding::Motor(channel) => channel.apply_backlog(),
            CapabilityBinding::Encoder(_)
            | CapabilityBinding::Imu(_)
            | CapabilityBinding::Accelerometer(_)
            | CapabilityBinding::Gyroscope(_)
            | CapabilityBinding::Range(_)
            | CapabilityBinding::Camera(_)
            | CapabilityBinding::Depth(_)
            | CapabilityBinding::Gnss(_)
            | CapabilityBinding::Magnetometer(_)
            | CapabilityBinding::Lidar(_)
            | CapabilityBinding::Mmwave(_)
            | CapabilityBinding::Microphone(_)
            | CapabilityBinding::Battery(_) => Ok(()),
        }
    }

    /// Leave this capability quiet when the simulation stops. Sensors have
    /// nothing to quiet.
    pub(crate) fn park(&mut self) -> Result<()> {
        match &mut self.binding {
            CapabilityBinding::Motor(channel) => channel.park(),
            CapabilityBinding::Encoder(_)
            | CapabilityBinding::Imu(_)
            | CapabilityBinding::Accelerometer(_)
            | CapabilityBinding::Gyroscope(_)
            | CapabilityBinding::Range(_)
            | CapabilityBinding::Camera(_)
            | CapabilityBinding::Depth(_)
            | CapabilityBinding::Gnss(_)
            | CapabilityBinding::Magnetometer(_)
            | CapabilityBinding::Lidar(_)
            | CapabilityBinding::Mmwave(_)
            | CapabilityBinding::Microphone(_)
            | CapabilityBinding::Battery(_) => Ok(()),
        }
    }

    /// Re-anchor sensor schedules and clear state derived from the previous
    /// world history before the first sample on a rewound timeline.
    pub(crate) fn reset(&mut self, logical_time_ns: u64) -> Result<()> {
        match &mut self.binding {
            CapabilityBinding::Encoder(channel) => channel.reset(logical_time_ns),
            CapabilityBinding::Imu(channel) => channel.reset(logical_time_ns),
            CapabilityBinding::Accelerometer(channel) => channel.reset(logical_time_ns),
            CapabilityBinding::Gyroscope(channel) => channel.reset(logical_time_ns),
            CapabilityBinding::Range(channel) => channel.reset(logical_time_ns),
            CapabilityBinding::Camera(channel) => channel.reset(logical_time_ns),
            CapabilityBinding::Depth(channel) => channel.reset(logical_time_ns),
            CapabilityBinding::Gnss(channel) => channel.reset(logical_time_ns),
            CapabilityBinding::Magnetometer(channel) => channel.reset(logical_time_ns),
            CapabilityBinding::Lidar(channel) => channel.reset(logical_time_ns),
            CapabilityBinding::Mmwave(channel) => channel.reset(logical_time_ns),
            CapabilityBinding::Microphone(channel) => channel.reset(logical_time_ns),
            CapabilityBinding::Battery(channel) => channel.device.reset(logical_time_ns),
            CapabilityBinding::Motor(_) => Ok(()),
        }
    }

    /// Read this capability for `step` and publish what it produced, when the
    /// step is one it publishes on. Actuators produce nothing.
    ///
    /// Simulated sensors read the world at exactly the instant it advanced to,
    /// so their capture is exact rather than uncertain. A battery reports what
    /// the pack is, not what a sensor saw at an instant, so it is state stamped
    /// with the world step like the clock itself.
    pub(crate) fn publish_due(
        &mut self,
        step: SensorStep,
        world_step: &WorldStepToken,
    ) -> Result<()> {
        let captured_at = CaptureStamp::exact(world_step.instant());
        match &mut self.binding {
            CapabilityBinding::Encoder(channel) => channel.publish_due(step, captured_at),
            CapabilityBinding::Imu(channel) => channel.publish_due(step, captured_at),
            CapabilityBinding::Accelerometer(channel) => channel.publish_due(step, captured_at),
            CapabilityBinding::Gyroscope(channel) => channel.publish_due(step, captured_at),
            CapabilityBinding::Range(channel) => channel.publish_due(step, captured_at),
            CapabilityBinding::Camera(channel) => channel.publish_due(step, captured_at),
            CapabilityBinding::Depth(channel) => channel.publish_due(step, captured_at),
            CapabilityBinding::Gnss(channel) => channel.publish_due(step, captured_at),
            CapabilityBinding::Magnetometer(channel) => channel.publish_due(step, captured_at),
            CapabilityBinding::Lidar(channel) => channel.publish_due(step, captured_at),
            CapabilityBinding::Mmwave(channel) => channel.publish_due(step, captured_at),
            CapabilityBinding::Microphone(channel) => channel.publish_due(step, captured_at),
            CapabilityBinding::Battery(channel) => {
                if let Some(state) = channel.device.read_if_due(step)? {
                    channel.publisher.publish(world_step, state)?;
                }
                Ok(())
            }
            CapabilityBinding::Motor(_) => Ok(()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use phoxal_bus::{
        BusMetadata, CodecId, ParticipantReadyStatus, ParticipantSourceIdentity, ProducerId,
        SourceAttribution,
    };

    fn producer(value: u128) -> ProducerId {
        ProducerId::try_from((1_u128 << 124) | value).expect("canonical test producer")
    }

    fn source(participant: &str, producer: ProducerId) -> ParticipantSourceIdentity {
        ParticipantSourceIdentity::new(
            ParticipantId::new(participant).expect("valid test participant"),
            producer,
        )
    }

    fn observed(body: u8, source: ParticipantSourceIdentity, sequence: u64) -> Observed<u8> {
        Observed {
            body,
            metadata: BusMetadata {
                codec: CodecId::MessagePack.as_u8(),
                sequence,
                stream_position: None,
                produced_at: None,
                source: SourceAttribution::Participant(source),
            },
            observed_at: LocalInstant::try_now().expect("test host clock"),
        }
    }

    #[test]
    fn drive_command_survives_a_wrong_source_flood_before_motor_selection() {
        let drive = ParticipantId::new(MOTOR_COMMAND_AUTHORITY).expect("valid drive participant");
        let drive_source = source(MOTOR_COMMAND_AUTHORITY, producer(1));
        let rogue_source = source("rogue", producer(2));
        let mut authority = FixedSourceLease::new(
            "component/motor/command",
            drive,
            MOTOR_SOURCE_SILENCE,
            Duration::MAX,
        );
        authority.update_ready(&drive_source, ParticipantReadyStatus::Ready);

        // Setpoint receive preserves one pending value per producer.  Drive's
        // command is therefore still present when a rogue producer has
        // flooded its own pending slot.
        admit_pending(
            &mut authority,
            vec![
                observed(7, drive_source, 1),
                observed(90, rogue_source.clone(), 1),
                observed(91, rogue_source.clone(), 2),
                observed(92, rogue_source, 3),
            ],
        );

        assert_eq!(
            authority.live_host(LocalInstant::try_now().expect("test host clock")),
            Some(&7)
        );
    }
}
