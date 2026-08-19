//! Motor capability: applies `component::motor::Command` to the Webots `Motor`
//! device.
//!
//! A Webots motor is position-controlled by default, so every velocity and
//! torque command first releases the position target to infinity. The declared
//! gear ratio converts the actuator-side command the contract carries into the
//! joint-side value Webots takes.

use anyhow::Result;
use phoxal::api;
use phoxal::model::component::capability::MotorCommand;
use phoxal::model::identity::CapabilityRef;
use std::fmt;

#[derive(Clone, Debug)]
pub(crate) struct MotorSpec {
    pub(crate) reference: CapabilityRef,
    pub(crate) actuator_type: MotorCommand,
    pub(crate) gear_ratio: f64,
}

pub(crate) struct NativeMotor {
    motor: webots_rs::device::motor::Motor,
    actuator_type: MotorCommand,
    gear_ratio: f64,
}

/// The command mode carried by the component wire enum, kept separate from
/// its numeric payload so a rejected mode mismatch cannot be mistaken for a
/// zero/stop command.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum WireMotorCommandMode {
    Position,
    Velocity,
    Torque,
    Stop,
}

/// The local actuation decision made before touching a Webots device.
///
/// Keeping this dispatch pure makes the important fail-closed property
/// testable without opening Webots: a mismatched command produces an error and
/// no action at all, while `Stop` is the only explicit park action.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum MotorAction {
    Position(f64),
    Velocity(f64),
    Torque(f64),
    Stop,
}

/// Why a motor command was refused locally before any Webots call.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) enum MotorCommandFault {
    ModeMismatch {
        configured: MotorCommand,
        received: WireMotorCommandMode,
    },
    NonFinite {
        mode: WireMotorCommandMode,
        value: f32,
    },
    InvalidGearRatio(f64),
}

impl fmt::Display for MotorCommandFault {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ModeMismatch {
                configured,
                received,
            } => write!(
                formatter,
                "motor configured for {configured:?} cannot accept {received:?} command"
            ),
            Self::NonFinite { mode, value } => {
                write!(formatter, "motor {mode:?} command is not finite: {value:?}")
            }
            Self::InvalidGearRatio(value) => {
                write!(
                    formatter,
                    "motor gear ratio must be finite and non-zero: {value}"
                )
            }
        }
    }
}

impl std::error::Error for MotorCommandFault {}

impl NativeMotor {
    pub(crate) fn new(webots: &webots_rs::Webots, spec: &MotorSpec) -> Result<Self> {
        Ok(Self {
            motor: webots.motor(spec.reference.to_string())?,
            actuator_type: spec.actuator_type,
            gear_ratio: spec.gear_ratio,
        })
    }

    pub(crate) fn apply(&self, command: &api::component::motor::Command) -> Result<()> {
        match dispatch(self.actuator_type, command, self.gear_ratio)? {
            MotorAction::Position(value) => self.motor.set_position(value)?,
            MotorAction::Velocity(value) => {
                self.motor.set_position(f64::INFINITY)?;
                self.motor.set_velocity(value)?;
            }
            MotorAction::Torque(value) => {
                self.motor.set_position(f64::INFINITY)?;
                self.motor.set_torque(value)?;
            }
            MotorAction::Stop => match self.actuator_type {
                MotorCommand::Position => self.motor.set_velocity(0.0)?,
                MotorCommand::Velocity => {
                    self.motor.set_position(f64::INFINITY)?;
                    self.motor.set_velocity(0.0)?;
                }
                MotorCommand::Torque => {
                    self.motor.set_position(f64::INFINITY)?;
                    self.motor.set_torque(0.0)?;
                }
            },
        }
        Ok(())
    }

    /// Leave the motor quiet: a simulation that stopped, or a step with no live
    /// authorised command, must not keep the wheel turning.
    pub(crate) fn stop(&self) -> Result<()> {
        self.apply(&api::component::motor::Command::Stop)
    }
}

fn dispatch(
    configured: MotorCommand,
    command: &api::component::motor::Command,
    gear_ratio: f64,
) -> std::result::Result<MotorAction, MotorCommandFault> {
    let (received, value) = match command {
        api::component::motor::Command::Position(value) => {
            (WireMotorCommandMode::Position, Some(*value))
        }
        api::component::motor::Command::Velocity(value) => {
            (WireMotorCommandMode::Velocity, Some(*value))
        }
        api::component::motor::Command::Torque(value) => {
            (WireMotorCommandMode::Torque, Some(*value))
        }
        api::component::motor::Command::Stop => (WireMotorCommandMode::Stop, None),
    };

    let matches = matches!(
        (configured, received),
        (MotorCommand::Position, WireMotorCommandMode::Position)
            | (MotorCommand::Velocity, WireMotorCommandMode::Velocity)
            | (MotorCommand::Torque, WireMotorCommandMode::Torque)
            | (_, WireMotorCommandMode::Stop)
    );
    if !matches {
        return Err(MotorCommandFault::ModeMismatch {
            configured,
            received,
        });
    }

    let Some(value) = value else {
        return Ok(MotorAction::Stop);
    };
    if !value.is_finite() {
        return Err(MotorCommandFault::NonFinite {
            mode: received,
            value,
        });
    }
    if !(gear_ratio.is_finite() && gear_ratio != 0.0) {
        return Err(MotorCommandFault::InvalidGearRatio(gear_ratio));
    }
    let value = actuator_to_joint_value(f64::from(value), gear_ratio);
    if !value.is_finite() {
        return Err(MotorCommandFault::NonFinite {
            mode: received,
            value: value as f32,
        });
    }
    Ok(match received {
        WireMotorCommandMode::Position => MotorAction::Position(value),
        WireMotorCommandMode::Velocity => MotorAction::Velocity(value),
        WireMotorCommandMode::Torque => MotorAction::Torque(value),
        WireMotorCommandMode::Stop => MotorAction::Stop,
    })
}

fn actuator_to_joint_value(actuator_value: f64, gear_ratio: f64) -> f64 {
    actuator_value / gear_ratio
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn actuator_to_joint_scales_by_gear_ratio() {
        assert_eq!(actuator_to_joint_value(4.0, 2.0), 2.0);
    }

    #[test]
    fn position_dispatch_scales_and_targets_a_position_motor() {
        assert_eq!(
            dispatch(
                MotorCommand::Position,
                &api::component::motor::Command::Position(4.0),
                2.0,
            ),
            Ok(MotorAction::Position(2.0))
        );
    }

    #[test]
    fn every_non_stop_mode_mismatch_is_rejected_before_actuation() {
        for (configured, command) in [
            (
                MotorCommand::Position,
                api::component::motor::Command::Velocity(1.0),
            ),
            (
                MotorCommand::Position,
                api::component::motor::Command::Torque(1.0),
            ),
            (
                MotorCommand::Velocity,
                api::component::motor::Command::Position(1.0),
            ),
            (
                MotorCommand::Velocity,
                api::component::motor::Command::Torque(1.0),
            ),
            (
                MotorCommand::Torque,
                api::component::motor::Command::Position(1.0),
            ),
            (
                MotorCommand::Torque,
                api::component::motor::Command::Velocity(1.0),
            ),
        ] {
            assert!(matches!(
                dispatch(configured, &command, 1.0),
                Err(MotorCommandFault::ModeMismatch { .. })
            ));
        }
    }

    #[test]
    fn stop_is_the_explicit_park_action_for_every_mode() {
        for configured in [
            MotorCommand::Position,
            MotorCommand::Velocity,
            MotorCommand::Torque,
        ] {
            assert_eq!(
                dispatch(configured, &api::component::motor::Command::Stop, 0.0),
                Ok(MotorAction::Stop)
            );
        }
    }
}
