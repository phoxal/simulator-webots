//! Magnetometer capability: publishes `component::magnetometer::Sample` from
//! the Webots `Compass` device, which reports the world's north vector in the
//! sensor's own frame.

use phoxal::api;

use super::vector_sensor;

vector_sensor!(
    NativeMagnetometer,
    webots_rs::device::compass::Compass,
    compass,
    api::component::magnetometer::Sample,
    magnetic_field,
);
