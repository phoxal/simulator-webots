//! Accelerometer capability: publishes `component::accelerometer::Sample` from
//! the Webots `Accelerometer` device, which reports proper acceleration in the
//! sensor's own frame.

use phoxal::api;

use super::vector_sensor;

vector_sensor!(
    NativeAccelerometer,
    webots_rs::device::accelerometer::Accelerometer,
    accelerometer,
    api::component::accelerometer::Sample,
    linear_acceleration,
);
