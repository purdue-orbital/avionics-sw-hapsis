#![no_std]

/// Time stamped barometer data structure
#[derive(Copy, Clone, Default)]
pub struct BaroData {
    pub pressure: f32,
    pub temperature: f32,
    pub time_stamp: u32,
}

impl BaroData {
    // Little endian representation of the struct above to be written to a binary file
    pub fn as_bytes(&self) -> [u8; 12] {
        let mut buf_index = 0;
        let mut buf: [u8;12] = [0u8; 12];
        let pressure_bytes: [u8; 4] = self.pressure.to_le_bytes();
        let temperature_bytes: [u8; 4] = self.temperature.to_le_bytes();
        let time_stamp_bytes: [u8; 4] = self.time_stamp.to_le_bytes();
         for byte in pressure_bytes {
            buf[buf_index] = byte;
            buf_index += 1;
        }
        for byte in temperature_bytes {
            buf[buf_index] = byte;
            buf_index += 1;
        }
        for byte in time_stamp_bytes {
            buf[buf_index] = byte;
            buf_index += 1;
        }

        buf
    }

    // Hard coded Byte length
    pub const fn byte_len() -> usize {
        12
    }
}

/// Time stamped imu data structure
#[derive(Copy, Clone, Default)]
pub struct ImuData {
    pub acceleration: [f32; 3],
    pub gyro: [f32; 3],
    pub mag: [f32; 3],
    pub time_stamp: u32,
}

impl ImuData {
    // Little Endian Representaiton of the ImuData to be written to a binary file
    pub fn as_bytes(&self) -> [u8; 40] {
        let mut buf_index = 0;
        let mut buf: [u8;40] = [0u8; 40];
        for i in 0..3 {
            for byte in self.acceleration[i].to_le_bytes() {
                buf[buf_index] = byte;
                buf_index += 1;
            }
        }

        for i in 0..3 {
            for byte in self.gyro[i].to_le_bytes() {
                buf[buf_index] = byte;
                buf_index += 1;
            }
        }

        for i in 0..3 {
            for byte in self.mag[i].to_le_bytes() {
                buf[buf_index] = byte;
                buf_index += 1;
            }
        }

        for byte in self.time_stamp.to_le_bytes() {
            buf[buf_index] = byte;
            buf_index += 1;
        }

        buf
    }

    // Hard coded length of the ImuData Struct in Bytes 
    pub const fn byte_len() -> usize {
        40
    }
}