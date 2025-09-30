#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::{Spawner, task};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::{
    Duration, Instant, Timer, WithTimeout
};
use embassy_sync::{
    channel::Channel,
    mutex::Mutex,
    blocking_mutex::raw::ThreadModeRawMutex,
};
use avionics_sw_hapsis::*;
use {defmt_rtt as _, panic_probe as _};

use libm::powf;

static BARO_DATA_CHANNEL: Channel<ThreadModeRawMutex, BaroData, 4> = Channel::new(); // baro data to send to sd card
static BARO_ALT_CHANNEL: Channel<ThreadModeRawMutex, f32, 4> = Channel::new(); // filtered altitude to send to control task
static IMU_DATA_CHANNEL: Channel<ThreadModeRawMutex, ImuData, 4> = Channel::new(); // imu data to send to sd card and gnc

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World!");

    let led = Output::new(p.PB7, Level::High, Speed::Low);

    _spawner.spawn(control_task(led)).unwrap();
    _spawner.spawn(baro_task()).unwrap();
    _spawner.spawn(imu_task()).unwrap();
    _spawner.spawn(log_task()).unwrap();

    info!("All tasks spawned");
}


#[task]
async fn control_task(mut led: Output<'static>) {

    info!("Starting main control loop");

    loop {
        // do control stuff here

        // blink led to show alive
        led.set_low();

        if let Ok(alt) = BARO_ALT_CHANNEL.try_receive() {
            info!("Current altitude: {} m", alt);
        }

        Timer::after(Duration::from_millis(100)).await;
    }

}

// barometer data acquisition, timestamping, and altitude filtering task
// reads sensor data, filters altitude to ensure proper launch procedure followed in control task
// sends filtered data to control task at low rate (1Hz or so)
// sends data to logging task at higher rate (10-20Hz)
#[task]
async fn baro_task() {
    info!("Starting barometer task");

    // altitude filter buffer
    // we start at 0m altitude so we don't need to fill the buffer with initial values
    let mut alt_buffer: [f32; 10] = [0.0; 10];

    loop {
        // fake data
        let time_stamp = Instant::now().as_micros() as u32;
        let data = BaroData {
            pressure: 1013.25,
            temperature: 25.0,
            time_stamp: time_stamp,
        };

        // try sending data, if channel is full, flush it and send again
        match BARO_DATA_CHANNEL.try_send(data) {
            Ok(_) => {
                info!("sent baro data: p: {}, t: {}, ts: {}", data.pressure, data.temperature, data.time_stamp);
            }
            Err(_) => {
                warn!("baro data channel full, flushing data");
                BARO_DATA_CHANNEL.clear();

                // if queue is empty wait until we can send until timeout
                BARO_DATA_CHANNEL.send(data).with_timeout(Duration::from_millis(200)).await.ok(); 
            }
        }

        alt_buffer.rotate_right(1);
        alt_buffer[0] = 44330.0 * (1.0 - powf(data.pressure / 1013.25, 1.0 / 5.255));

        // filter altitide
        // ex: rolling average
        let alt_sum: f32 = alt_buffer.iter().sum();
        let alt_avg: f32 = alt_sum / alt_buffer.len() as f32;

        // try sending filtered altitude, if channel is full, flush it and send again
        match BARO_ALT_CHANNEL.try_send(alt_avg) {
            Ok(_) => {
                info!("sent filtered altitude: {}", alt_avg);
            }
            Err(_) => {
                warn!("baro alt channel full, flushing data");
                BARO_ALT_CHANNEL.clear();
                BARO_ALT_CHANNEL.send(alt_avg).with_timeout(Duration::from_millis(200)).await.ok();
            }
        };

        // no need for perfectly timed data, simple delay is fine
        Timer::after(Duration::from_millis(500)).await;
    }
}

// imu data acquisition and timestamping. Most likely no filtering is needed
// sends data to GNC can bus task at high rate (50-100Hz, or whatever GNC needs)
// sends data to logging task at higher rate (10-20Hz)
#[task]
async fn imu_task() {
    info!("Starting barometer task");

    loop {
        // fake data
        let time_stamp = Instant::now().as_micros() as u32;
        let data = ImuData {
            acceleration: [0.0, 0.0, 9.81],
            gyro: [0.0, 0.0, 0.0],
            mag: [0.0, 0.0, 0.0],
            time_stamp: time_stamp,
        };

        // try sending data, if channel is full, flush it and send again
        match IMU_DATA_CHANNEL.try_send(data) {
            Ok(_) => {
                info!("sent imu data: a: ({}, {}, {}), g: ({}, {}, {}), m: ({}, {}, {}), ts: {}", 
                    data.acceleration[0], data.acceleration[1], data.acceleration[2],
                    data.gyro[0], data.gyro[1], data.gyro[2],
                    data.mag[0], data.mag[1], data.mag[2],
                    data.time_stamp);
            }
            Err(_) => {
                warn!("imu data channel full, flushing data");
                IMU_DATA_CHANNEL.clear();

                // if queue is empty wait until we can send until timeout
                IMU_DATA_CHANNEL.send(data).with_timeout(Duration::from_millis(50)).await.ok(); 
            }
        };

        // no need for perfectly timed data, simple delay is fine
        Timer::after(Duration::from_millis(500)).await;
    }
}

// receives sensor data, adds to byte buffer. Once buffer reaches 256 bytes writes data to sd card
#[task]
async fn log_task() {
    info!("Entered logging task");

    let mut buf_index: u16 = 0;

    loop {
        // check for baro data
        while let Ok(data) = BARO_DATA_CHANNEL.try_receive() {
            info!("received baro data: p: {}, t: {}, ts: {}", data.pressure, data.temperature, data.time_stamp);

            // add to byte buffer
            buf_index += 12;
        }
        
        while let Ok(data) = IMU_DATA_CHANNEL.try_receive() {
            info!("received imu data: a: ({}, {}, {}), g: ({}, {}, {}), m: ({}, {}, {}), ts: {}", 
                data.acceleration[0], data.acceleration[1], data.acceleration[2],
                data.gyro[0], data.gyro[1], data.gyro[2],
                data.mag[0], data.mag[1], data.mag[2],
                data.time_stamp);

                // add to byte buffer
                buf_index += 40;
        }

        // if byte buffer has 256 bytes, send to sd card
        if buf_index >= 256 {
            info!("buffer full, writing to sd card");
            buf_index -= 256;
        }
    
        // wait state to let other tasks run
        Timer::after(Duration::from_millis(50)).await;

    }
}