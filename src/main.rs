#![no_std]
#![no_main]

use chrono::format;
use defmt::*;
use embassy_executor::{Spawner, task};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_time::{
    Duration, Instant, Timer, WithTimeout, Delay
};
use embassy_sync::{
    channel::Channel,
    blocking_mutex::raw::ThreadModeRawMutex,
};
use avionics_sw_hapsis::*;
use {defmt_rtt as _, panic_probe as _};

use embassy_stm32::spi::{BitOrder, Spi};
use embassy_stm32::time::Hertz;
use {defmt_rtt as _, panic_probe as _};
use embedded_sdmmc_async::{sdcard::{self, AcquireOpts}, File, Mode, SdCard, TimeSource, Timestamp, Volume, VolumeIdx, VolumeManager};
use embedded_hal_bus::spi::ExclusiveDevice;

use libm::powf;

static BARO_DATA_CHANNEL: Channel<ThreadModeRawMutex, BaroData, 4> = Channel::new(); // baro data to send to sd card
static BARO_ALT_CHANNEL: Channel<ThreadModeRawMutex, f32, 4> = Channel::new(); // filtered altitude to send to control task
static IMU_DATA_CHANNEL: Channel<ThreadModeRawMutex, ImuData, 4> = Channel::new(); // imu data to send to sd card and gnc

/// Code from https://github.com/rp-rs/rp-hal-boards/blob/main/boards/rp-pico/examples/pico_spi_sd_card.rs
/// A dummy timesource, which is mostly important for creating files.
#[derive(Default)]
pub struct DummyTimesource();

impl TimeSource for DummyTimesource {
    // In theory you could use the RTC of the rp2040 here, if you had
    // any external time synchronizing device.
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

const FILE_TO_CREATE: &str = "CREATE.TXT";
const BUFFER_MAX_LENGTH: usize = 256;

const MAX_DIRS: usize = 4;
const MAX_FILES: usize = 4;
const MAX_VOLUMES: usize = 1;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p: embassy_stm32::Peripherals = embassy_stm32::init(Default::default());
    info!("Hello World!");

    let mut spi_config = embassy_stm32::spi::Config::default();
    spi_config.frequency = Hertz(12_000_000);
    spi_config.bit_order = BitOrder::MsbFirst;
    spi_config.gpio_speed = Speed::Low;

    let sck = p.PB13;
    let mosi = p.PB15;
    let miso = p.PB14;
    let dma_tx = p.DMA1_CH4;
    let dma_rx = p.DMA1_CH3;

    let spi = Spi::new(p.SPI2, 
        sck, 
        mosi, 
        miso,
        dma_tx, 
        dma_rx, 
        spi_config
    );

    let spi_cs = Output::new(p.PA3, Level::High, Speed::Low);
    let spi_dev = ExclusiveDevice::new(spi, spi_cs, Delay).unwrap();

    let sd_card_options = AcquireOpts{use_crc: true, acquire_retries: 5};

    let sd_card = SdCard::new_with_options(spi_dev, Delay, sd_card_options);

    info!("Init SD card controller and retrieve card size...");
    let sd_size = sd_card.num_bytes().await;

    match(sd_size) {
        Ok(size) => {
            info!("SD Card Size: {}", size);
        },
        Err(e) => {
            info!("Error: {:?}", defmt::Debug2Format(&e));
        }
    }

    let volume_mgr: VolumeManager<SdCard<ExclusiveDevice<Spi<'_, embassy_stm32::mode::Async>, Output<'_>, Delay>, Delay>, DummyTimesource> = VolumeManager::new(sd_card, DummyTimesource::default());
    // let mut volume: embedded_sdmmc_async::Volume<'_, SdCard<ExclusiveDevice<Spi<'_, embassy_stm32::mode::Async>, Output<'_>, Delay>, Delay>, DummyTimesource, 4, 4, 1> = volume_mgr.open_volume(VolumeIdx(0)).await.unwrap();

    // let root_dir = volume.open_root_dir().unwrap();
    // info!("\nCreating file {}...", FILE_TO_CREATE);

    // if (!root_dir.find_directory_entry(FILE_TO_CREATE).is_err()) {
    //         info!("File with name {} already exists!", FILE_TO_CREATE);
    //         root_dir.delete_file_in_dir(FILE_TO_CREATE).unwrap();
    //         info!("Deleting File {}", FILE_TO_CREATE);

    //     }
    // let f: File<'_, SdCard<ExclusiveDevice<Spi<'_, embassy_stm32::mode::Async>, Output<'_>, Delay>, Delay>, DummyTimesource, 4, 4, 1> = root_dir.open_file_in_dir(FILE_TO_CREATE, Mode::ReadWriteCreate).unwrap();
    // let mut start_timestamp: u32 = Instant::now().as_micros() as u32;
    // f.write(&[0u8; 65536]).unwrap();
    
    // let mut end_timestamp: u32 = Instant::now().as_micros() as u32;
    // info!("Diff: {}", end_timestamp - start_timestamp);

    // f.close().unwrap();
    let led = Output::new(p.PB7, Level::High, Speed::Low);

    _spawner.spawn(control_task(led)).unwrap();
    _spawner.spawn(baro_task()).unwrap();
    _spawner.spawn(imu_task()).unwrap();
    _spawner.spawn(log_task(volume_mgr)).unwrap();

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
        let time_stamp: u32 = Instant::now().as_micros() as u32;
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
async fn log_task(volume_mgr: VolumeManager<SdCard<ExclusiveDevice<Spi<'static, embassy_stm32::mode::Async>, Output<'static>, Delay>, Delay>, DummyTimesource>) {
    info!("Entered logging task");

    let mut buf_index: usize = 0;
    let mut buf: [u8; BUFFER_MAX_LENGTH] = [0u8; BUFFER_MAX_LENGTH];

    loop {
        // check for baro data
        while let Ok(data) = BARO_DATA_CHANNEL.try_receive() {
            info!("received baro data: p: {}, t: {}, ts: {}", data.pressure, data.temperature, data.time_stamp);
            let pressure: [u8; 4] = data.pressure.to_le_bytes();
            let temperature: [u8; 4] = data.temperature.to_le_bytes();
            let timestamp: [u8; 4] = data.time_stamp.to_le_bytes();
            for byte in pressure {
                buf[buf_index] = byte;
                buf_index += 1;
            }
            for byte in temperature {
                buf[buf_index] = byte;
                buf_index += 1;
            }
            for byte in timestamp {
                buf[buf_index] = byte;
                buf_index += 1;
            }
        }
        
        while let Ok(data) = IMU_DATA_CHANNEL.try_receive() {
            info!("received imu data: a: ({}, {}, {}), g: ({}, {}, {}), m: ({}, {}, {}), ts: {}", 
                data.acceleration[0], data.acceleration[1], data.acceleration[2],
                data.gyro[0], data.gyro[1], data.gyro[2],
                data.mag[0], data.mag[1], data.mag[2],
                data.time_stamp);
            let imu_line = heapless::format!(100; "received imu data: a: ({}, {}, {}), g: ({}, {}, {}), m: ({}, {}, {}), ts: {}\r\n", 
                data.acceleration[0], data.acceleration[1], data.acceleration[2],
                data.gyro[0], data.gyro[1], data.gyro[2],
                data.mag[0], data.mag[1], data.mag[2],
                data.time_stamp).unwrap();


            write_to_sd_card(&volume_mgr, imu_line.as_str()).await;

            for i in 0..3 {
                for byte in data.acceleration[i].to_le_bytes() {
                    buf[buf_index] = byte;
                    buf_index += 1;
                }
            }

            for i in 0..3 {
                for byte in data.gyro[i].to_le_bytes() {
                    buf[buf_index] = byte;
                    buf_index += 1;
                }
            }

            for i in 0..3 {
                for byte in data.mag[i].to_le_bytes() {
                    buf[buf_index] = byte;
                    buf_index += 1;
                }
            }

            for byte in data.time_stamp.to_le_bytes() {
                
                buf[buf_index] = byte;
                buf_index += 1;
            }
        }

        // // if byte buffer has 256 bytes, send to sd card
        // if buf_index >= BUFFER_MAX_LENGTH {
        // info!("buffer full, writing to sd card");
        // write_to_sd_card(& volume_mgr, buf).await;
        buf_index = 0;
        // }
    
        // wait state to let other tasks run
        Timer::after(Duration::from_millis(50)).await;

    }
}

async fn write_to_sd_card(volume_mgr: &  VolumeManager<SdCard<ExclusiveDevice<Spi<'_, embassy_stm32::mode::Async>, Output<'_>, Delay>, Delay>, DummyTimesource>, buf: &str) {
    let volume_future = volume_mgr.open_volume(VolumeIdx(0)).await;

    match volume_future {
        Ok(volume) => {
            let root_dir = volume.open_root_dir().await.unwrap();
            info!("\nCreating or Appending file {}...", FILE_TO_CREATE);

            // if (!root_dir.find_directory_entry(FILE_TO_CREATE).is_err()) {
            //         info!("File with name {} already exists!", FILE_TO_CREATE);
            //         root_dir.delete_file_in_dir(FILE_TO_CREATE).unwrap();
            //         info!("Deleting File {}", FILE_TO_CREATE);
            //     }
            let f: File<'_, SdCard<ExclusiveDevice<Spi<'_, embassy_stm32::mode::Async>, Output<'_>, Delay>, Delay>, DummyTimesource, 4, 4, 1> = root_dir.open_file_in_dir(FILE_TO_CREATE, Mode::ReadWriteCreateOrAppend).await.unwrap();
            let mut start_timestamp: u32 = Instant::now().as_micros() as u32;
            match f.write(buf.as_bytes()).await {
                Ok(_) => {
                    let mut end_timestamp: u32 = Instant::now().as_micros() as u32;
                    info!("Diff: {}", end_timestamp - start_timestamp);
                },

                Err(e) => {
                    let mut end_timestamp: u32 = Instant::now().as_micros() as u32;
                    error!("Diff: {}", end_timestamp - start_timestamp);
                    
                }
            }
            
            f.close().await.unwrap();
            root_dir.close().unwrap();
            volume.close().await.unwrap();
        },

        Err(e) => {
            error!("{}", defmt::Debug2Format(&e));
            match (&e) {
                embedded_sdmmc_async::Error::TooManyOpenVolumes=>{}
                embedded_sdmmc_async::Error::DeviceError(_) => {},
                embedded_sdmmc_async::Error::FormatError(_) => {},
                embedded_sdmmc_async::Error::NoSuchVolume => {},
                embedded_sdmmc_async::Error::FilenameError(filename_error) => {},
                embedded_sdmmc_async::Error::TooManyOpenDirs => {},
                embedded_sdmmc_async::Error::TooManyOpenFiles => {},
                embedded_sdmmc_async::Error::BadHandle => {},
                embedded_sdmmc_async::Error::NotFound => {},
                embedded_sdmmc_async::Error::FileAlreadyOpen => {},
                embedded_sdmmc_async::Error::DirAlreadyOpen => {},
                embedded_sdmmc_async::Error::OpenedDirAsFile => {},
                embedded_sdmmc_async::Error::OpenedFileAsDir => {},
                embedded_sdmmc_async::Error::DeleteDirAsFile => {},
                embedded_sdmmc_async::Error::VolumeStillInUse => {},
                embedded_sdmmc_async::Error::VolumeAlreadyOpen => {},
                embedded_sdmmc_async::Error::Unsupported => {},
                embedded_sdmmc_async::Error::EndOfFile => {},
                embedded_sdmmc_async::Error::BadCluster => {},
                embedded_sdmmc_async::Error::ConversionError => {},
                embedded_sdmmc_async::Error::NotEnoughSpace => {},
                embedded_sdmmc_async::Error::AllocationError => {},
                embedded_sdmmc_async::Error::UnterminatedFatChain => {},
                embedded_sdmmc_async::Error::ReadOnly => {},
                embedded_sdmmc_async::Error::FileAlreadyExists => {},
                embedded_sdmmc_async::Error::BadBlockSize(_) => {},
                embedded_sdmmc_async::Error::InvalidOffset => {},
                embedded_sdmmc_async::Error::DiskFull => {},
                embedded_sdmmc_async::Error::DirAlreadyExists => {},
                embedded_sdmmc_async::Error::LockError => {},
            }
        }
    }

    
}
