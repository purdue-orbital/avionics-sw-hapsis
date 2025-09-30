#![no_std]

/// Time stamped barometer data structure
#[derive(Copy, Clone)]
pub struct BaroData {
    pub pressure: f32,
    pub temperature: f32,
    pub time_stamp: u32,
}

/// Time stamped imu data structure
#[derive(Copy, Clone)]
pub struct ImuData {
    pub acceleration: [f32; 3],
    pub gyro: [f32; 3],
    pub mag: [f32; 3],
    pub time_stamp: u32,
}
