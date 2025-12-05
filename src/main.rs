#![no_std]
#![no_main]

use core::{cell::RefCell, error, fmt::Debug};

use defmt::*;
use embassy_executor::{Spawner, task};
use embassy_stm32::{bind_interrupts, gpio::{Level, Output, Speed}, i2c::{self, I2c}, peripherals};
use embassy_time::{
    Duration, Instant, Timer, WithTimeout, Delay
};
use embassy_sync::{
    blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex}, channel::Channel, mutex::{Mutex}
};
use avionics_sw_hapsis::*;
use futures_util::task::SpawnError;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

use embassy_stm32::spi::{BitOrder, Spi};
use embassy_stm32::time::Hertz;
use {defmt_rtt as _, panic_probe as _};
use embedded_sdmmc_async::{sdcard::{AcquireOpts}, Mode, SdCard, File, TimeSource, Timestamp, VolumeIdx, VolumeManager};
use bme280::i2c::AsyncBME280;
use adxl345_eh_driver::Driver as Adxl345Driver;

use libm::powf;

static BARO_DATA_CHANNEL_CAPACITY: usize = 100;
static IMU_DATA_CHANNEL_CAPACITY: usize = 100;


static BARO_DATA_CHANNEL: Channel<ThreadModeRawMutex, BaroData, BARO_DATA_CHANNEL_CAPACITY> = Channel::new(); // baro data to send to sd card
static BARO_ALT_CHANNEL: Channel<ThreadModeRawMutex, f32, BARO_DATA_CHANNEL_CAPACITY> = Channel::new(); // filtered altitude to send to control task
static IMU_DATA_CHANNEL: Channel<ThreadModeRawMutex, ImuData, IMU_DATA_CHANNEL_CAPACITY> = Channel::new(); // imu data to send to sd card and gnc

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

type FileType = File<'static, CriticalSectionRawMutex, SdCard<embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig<'static, CriticalSectionRawMutex, Spi<'static, embassy_stm32::mode::Async>, Output<'static>>, Delay>, DummyTimesource, 4, 4, 1>;
type VolumeRawMutexType = CriticalSectionRawMutex;
type VolumeManagerType = VolumeManager<VolumeRawMutexType, SdCard<embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig<'static, CriticalSectionRawMutex, Spi<'static, embassy_stm32::mode::Async>, Output<'static>>, Delay>, DummyTimesource>;
type I2cMutexType = Mutex<CriticalSectionRawMutex, I2c<'static, embassy_stm32::mode::Async, i2c::Master>>;

const IMU_FILENAME: &str = "IMU";
const BARO_FILENAME: &str = "BARO";

// Count of how many times IMU and Barometer has called a read/write to SD Card
static IMU_GLOBAL_COUNT: Mutex<CriticalSectionRawMutex, RefCell<u32>> = Mutex::new(RefCell::new(0));
static BARO_GLOBAL_COUNT: Mutex<CriticalSectionRawMutex, RefCell<u32>> = Mutex::new(RefCell::new(0));

// Create a Static Cell Guarding a Mutex that holds the reference to an SPI or I2C Bus so that the lifetime of the repsective bus can be global
static BUS: StaticCell<Mutex<CriticalSectionRawMutex, Spi<'static, embassy_stm32::mode::Async>>> = StaticCell::new();
static I2C_BUS: StaticCell<Mutex<CriticalSectionRawMutex, I2c<'static, embassy_stm32::mode::Async, i2c::Master>>> = StaticCell::new();

// Changes the rate at which the CPU gets data from the sensors
static BARO_MILLIS_DELAY: u64 = 20;
static IMU_MILLIS_DELAY: u64 = 20;

// Changes the rate at which the logging task polls from the channel
// This should be less than their corresponding sensor polling rate to limit the data channels being full
static BARO_POLL_MILLIS_DELAY: u64 = 1;
static IMU_POLL_MILLIS_DELAY: u64 = 1;

static BARO_MILLIS_TIMEOUT_DELAY: u64 = 100;

static BARO_SEND_TIMEOUT_MILLIS: u64 = 700;
static IMU_SEND_TIMEOUT_MILLIS: u64 = 700;

static SD_CARD_INIT_TIMEOUT_MILLIS: u64 = 1_000;
static SD_CARD_ERROR_INIT_TIMEOUT_MILLIS: u64 = 10_000;

static BME280_INIT_TIMEOUT_MILLIS: u64 = 750;
static ADXL345_INIT_TIMEOUT_MILLIS: u64 = 100;

static ADXL_I2C_ADDR: u8 = 0x53;
static BME_I2C_ADDR: u8 = 0x76;

static CONTROL_TASK_DELAY_MILLIS: u64 = 100;

// Used to initialize a static reference to the SDCard's Volume Manager
static VOLUME_MANAGER: StaticCell<VolumeManagerType> = StaticCell::new();

bind_interrupts!(struct Irqs {
    I2C2_EV => i2c::EventInterruptHandler<peripherals::I2C2>;
    I2C2_ER => i2c::ErrorInterruptHandler<peripherals::I2C2>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p: embassy_stm32::Peripherals = embassy_stm32::init(Default::default());
    info!("Hello World!");

    let i2c_config = embassy_stm32::i2c::Config::default();

    let sck = p.PB13;
    let mosi = p.PB15;
    let miso = p.PB14;
    let dma_tx = p.DMA1_CH4; 
    let dma_rx = p.DMA1_CH3;

    let mut spi_config: embassy_stm32::spi::Config = embassy_stm32::spi::Config::default();
    spi_config.frequency = Hertz(12_000_000);
    spi_config.bit_order = BitOrder::MsbFirst;
    spi_config.gpio_speed = Speed::Low;

    let spi = Spi::new(p.SPI2, 
        sck, 
        mosi, 
        miso,
        dma_tx, 
        dma_rx, 
        spi_config
    );

    let scl = p.PB10;
    let sda = p.PB11;
    let i2c_tx_dma = p.DMA1_CH7;
    let i2c_rx_dma = p.DMA1_CH2;

    let i2c = embassy_stm32::i2c::I2c::new(p.I2C2, scl, sda, Irqs, i2c_tx_dma, i2c_rx_dma, i2c_config);

    let i2c_mutex_guard = embassy_sync::mutex::Mutex::new(i2c);

    let spi_mutex_guard: &mut Mutex<CriticalSectionRawMutex, Spi<'static, embassy_stm32::mode::Async>> = BUS.init(embassy_sync::mutex::Mutex::new(spi));

    // SPI Chip Select Pin
    let spi_cs = Output::new(p.PA3, Level::High, Speed::Low);

    // I2C Bus
    let i2c_bus = I2C_BUS.init(i2c_mutex_guard);
   
    let led = Output::new(p.PB7, Level::High, Speed::Low);

    match _spawner.spawn(control_task(led)) {
        Ok(_) => {},
        Err(e) => {
            error!("Couldn't start Control Task. {}\n ", Debug2Format(&e));
        }
    };

    let spi_dev =  embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig::new(spi_mutex_guard, spi_cs, spi_config);

    // Set up the SD Card for initialization and allow cyclic redundancy check
    let sd_card_options = AcquireOpts{use_crc: true, acquire_retries: 5};

    let sd_card = SdCard::new_with_options(spi_dev, Delay, sd_card_options);

    // Start the Baro and IMU task for initialization
    match _spawner.spawn(baro_task(i2c_bus)) {
        Ok(_) => {},
        Err(e) => {
            error!("Couldn't start the barometer task. {}", Debug2Format(&e));
        },
    }
    match _spawner.spawn(imu_task(i2c_bus)) {
        Ok(_) => {},
        Err(e) => {
            error!("Couldn't start the imu task. {}", Debug2Format(&e));
        },
    };

    info!("Init SD card controller and retrieve card size...");
    loop {
        match sd_card.num_bytes().with_timeout(Duration::from_millis(SD_CARD_INIT_TIMEOUT_MILLIS)).await {
            Ok(timeout_res) => {
                match timeout_res {
                    Ok(size) => {
                        info!("SD Card Size: {}", size);
                        break;
                    },
                    Err(e) => {
                        error!("SD Card Error: {}", Debug2Format(&e));
                        // When an SD Card is missing I want a little more delay in checking for Initializing the SD Card Again so as to not busy up the CPU.
                        Timer::after(Duration::from_millis(SD_CARD_ERROR_INIT_TIMEOUT_MILLIS)).await;
                    }
                }
                
            },
            Err(e) => {
                error!("Timeout SD Card Error: {:?}", defmt::Debug2Format(&e));
            }
        }
    }

    // Create the volume manager and share it among different tasks using a static reference
    let volume_mgr = VolumeManager::new(sd_card, DummyTimesource::default());
    let long_live_volume_manager = VOLUME_MANAGER.init(volume_mgr);

    // Create the logging tasks after the SD Card has been initialized
    match _spawner.spawn(log_baro_task(long_live_volume_manager)) {
        Ok(_) => {},
        Err(e) => {
            error!("Couldn't spawn logging baro task! {}", Debug2Format(&e));
        },
    }
    match _spawner.spawn(log_imu_task(long_live_volume_manager)) {
        Ok(_) => {},
        Err(e) => {
            error!("Couldn't spawn logging imu task! {}", Debug2Format(&e));
        },
    }

    info!("Setup Complete");
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

        Timer::after(Duration::from_millis(CONTROL_TASK_DELAY_MILLIS)).await;
    }

}

// barometer data acquisition, timestamping, and altitude filtering task
// reads sensor data, filters altitude to ensure proper launch procedure followed in control task
// sends filtered data to control task at low rate (1Hz or so)
// sends data to logging task at higher rate (10-20Hz)
#[task]
async fn baro_task(i2c_mutex_bus: &'static I2cMutexType) {
    info!("Starting barometer task");

    // Initialize BME280 I2C Driver forever with a timeout after each error, so other parts of the program can continue
    // Initialize I2C Bus
    
    let bme_i2c_dev = embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice::new(i2c_mutex_bus);
    

    info!("Initializing BME 280");
    let mut bme280_dev = AsyncBME280::new(bme_i2c_dev, BME_I2C_ADDR);
    loop {
        match bme280_dev.init(& mut Delay).with_timeout(Duration::from_millis(BME280_INIT_TIMEOUT_MILLIS)).await {
            Ok(_) => {
                info!("BME 280 Loaded Successfully!");
                break;
            },
            Err(e) => {
                error!("BME 280 Error! {}", Debug2Format(&e));
                Timer::after_millis(BARO_MILLIS_TIMEOUT_DELAY).await;
            },
        }
    }

    // altitude filter buffer
    // we start at 0m altitude so we don't need to fill the buffer with initial values
    let mut alt_buffer: [f32; 10] = [0.0; 10];


    loop {
        
        // Read data from the BME 280
        match bme280_dev.measure(& mut Delay).await {
            Ok(measurement) => {
                let time_stamp: u32 = Instant::now().as_micros() as u32;
                let data = BaroData {
                    pressure: measurement.pressure,
                    temperature: measurement.temperature,
                    time_stamp: time_stamp,
                };

                // try sending data, if channel is full, flush it and send again
                match BARO_DATA_CHANNEL.try_send(data) {
                    Ok(_) => {
                        info!("sent baro data: p: {}, t: {}, ts: {}", data.pressure, data.temperature, data.time_stamp);
                    }
                    Err(_) => {
                        warn!("baro data channel full, sending again");
                        // if queue is empty wait until we can send until timeout
                        BARO_DATA_CHANNEL.send(data).with_timeout(Duration::from_millis(BARO_SEND_TIMEOUT_MILLIS)).await.ok(); 
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
                        warn!("baro alt channel full, sending again.");
                        BARO_ALT_CHANNEL.send(alt_avg).with_timeout(Duration::from_millis(BARO_SEND_TIMEOUT_MILLIS)).await.ok();
                    }
                };
            },

            Err(e) => {
                error!("BME Read Error! {}", Debug2Format(&e))
            }
        }
        // no need for perfectly timed data, simple delay is fine
        Timer::after(Duration::from_millis(BARO_MILLIS_DELAY)).await;
    }
}

// imu data acquisition and timestamping. Most likely no filtering is needed
// sends data to GNC can bus task at high rate (50-100Hz, or whatever GNC needs)
// sends data to logging task at higher rate (10-20Hz)
#[task]
async fn imu_task(i2c_bus: &'static I2cMutexType) {
    info!("Starting imu task");

    // Initialize ADXL 345 I2C Device
    // Initialize I2C Bus
    let adxl_i2c = embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice::new(i2c_bus);

    // 0x1D is the alt address depending on grounding of SDO or not in my case it is grounded
    let mut adxl345 = match Adxl345Driver::new(adxl_i2c, Some(ADXL_I2C_ADDR)).await {
        Ok(adxl_driver) => {
            adxl_driver
        },
        Err(e) => {
            error!("Couldn't instantiate ADXL Driver! {}", Debug2Format(&e));
            return;
        },
    }; 

    loop {
        info!("Initializing ADXL345");
        match adxl345.init().with_timeout(Duration::from_millis(ADXL345_INIT_TIMEOUT_MILLIS)).await {
            Ok(_) => {
                info!("Initialized ADXL345 Driver!");
                break;
            },
            Err(e) => {
                error!("ADXL345 Device Inititialization Error! {}", Debug2Format(&e));
            },
        }
    }   
    

    loop {

        let time_stamp: u32 = Instant::now().as_micros() as u32;

        // Get the acceleration reading from sensor
        match adxl345.get_accel().await {
            Ok(data) => {
                let data = ImuData {
                    acceleration: [data.0, data.1, data.2],
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
                        // IMU_DATA_CHANNEL.clear();

                        // if queue is empty wait until we can send until timeout
                        IMU_DATA_CHANNEL.send(data).with_timeout(Duration::from_millis(IMU_SEND_TIMEOUT_MILLIS)).await.ok(); 
                    }
                };
            },

            Err(e) => {
                error!("ADXL345 Read Error: {}", Debug2Format(&e));
            }
        }
        
        // no need for perfectly timed data, simple delay is fine
        Timer::after(Duration::from_millis(IMU_MILLIS_DELAY)).await;
    }
}

#[task]
// receives sensor data, adds to byte buffer. Once buffer reaches 256 bytes writes data to sd card
async fn log_baro_task(volume_mgr: &'static VolumeManagerType) {
    info!("Entered baro logging task");

    let mut buf_index: usize = 0;
    let mut byte_buf: [u8; BARO_DATA_CHANNEL_CAPACITY * BaroData::byte_len()] = [0u8; BARO_DATA_CHANNEL_CAPACITY * BaroData::byte_len()];

    loop {
        // check for baro data in the data channel until the buffer is full
        loop {
            match BARO_DATA_CHANNEL.try_receive() {
                Ok(data) => {
                    info!("received baro data: p: {}, t: {}, ts: {}", data.pressure, data.temperature, data.time_stamp);

                    for baro_byte in  data.as_bytes() {
                        byte_buf[buf_index] = baro_byte;
                        buf_index += 1;
                    }

                    if buf_index >= byte_buf.len() {
                        break;
                    }

                }

                Err(_) => {

                }
            }

            // Delay after receiving something to give Baro sensor task a chance to populate the buffer
            Timer::after(Duration::from_millis(BARO_POLL_MILLIS_DELAY)).await;
        }

        // Write the in memory buffer to the Barometer filename
        write_to_sd_card_buffer(volume_mgr, &byte_buf, BARO_FILENAME).await;
        buf_index = 0;

        {
            let baro_lock = BARO_GLOBAL_COUNT.lock().await;
            let mut baro_count_ref = baro_lock.borrow_mut();
            *baro_count_ref += 1;
            info!("Wrote Baro Data to SD Card {} Times!\n", *baro_count_ref);
            
        } 

    }
}

// Logs data from the IMU Data Channel to the SD Card
#[task]
async fn log_imu_task(volume_mgr: &'static VolumeManagerType) {
    info!("Entered IMU Logging Task");
    let mut buf_index: usize = 0;
    let mut byte_buf: [u8; IMU_DATA_CHANNEL_CAPACITY * ImuData::byte_len()] = [0u8; IMU_DATA_CHANNEL_CAPACITY * ImuData::byte_len()];

    loop {
        // Poll the IMU Data Channel until the in task buffer is full
        loop {
            
            match IMU_DATA_CHANNEL.try_receive() {
                Ok(data) => {
                    info!("received imu data: a: ({}, {}, {}), g: ({}, {}, {}), m: ({}, {}, {}), ts: {}", 
                        data.acceleration[0], data.acceleration[1], data.acceleration[2],
                        data.gyro[0], data.gyro[1], data.gyro[2],
                        data.mag[0], data.mag[1], data.mag[2],
                        data.time_stamp);


                    for imu_byte in  data.as_bytes() {
                        byte_buf[buf_index] = imu_byte;
                        buf_index += 1;
                    }

                    if buf_index >= IMU_DATA_CHANNEL_CAPACITY * ImuData::byte_len() {
                        break;
                    }

                },

                Err(_) => {

                }
            }

            // Write the in memory buffer to the Barometer filename
            Timer::after(Duration::from_millis(IMU_POLL_MILLIS_DELAY)).await;
        }
            
        // Write the in memory byte array buffer to the IMU Filename
        write_to_sd_card_buffer(volume_mgr, &byte_buf, IMU_FILENAME).await;

        buf_index = 0;
        {
            let imu_lock = IMU_GLOBAL_COUNT.lock().await;
            let mut imu_count_ref = imu_lock.borrow_mut();
            *imu_count_ref += 1;
            info!("Wrote IMU Data to SD Card {} Times!\n", *imu_count_ref);
            
        }
        
    }

}

// Writes a string to a file from the SD Card assuming the file is already open
// async fn write_to_file(file: &'static FileType, buf: &str) {
//     let start_timestamp: u32 = Instant::now().as_micros() as u32;
//     loop {
//         match file.write(buf.as_bytes()).await {
//                     Ok(_) => {
//                         let end_timestamp: u32 = Instant::now().as_micros() as u32;
//                         info!("Diff: {}", end_timestamp - start_timestamp);
//                         file.flush().await.unwrap();
//                         info!("Flushed to File!");
//                         break;
//                     },
                    
//                     Err(e) => {
//                         let end_timestamp: u32 = Instant::now().as_micros() as u32;
//                         error!("Diff: {}", end_timestamp - start_timestamp);
//                         error!("{}", defmt::Debug2Format(&e));
//                         Timer::after(Duration::from_micros(1_000_000)).await;
//                     }
//         }
//     }

// }

// Writes a string to a file from the SD Card with full opening and closing of the volume, directory, and file for the write operation
// async fn write_to_sd_card(volume_mgr: &  VolumeManagerType, buf: &str, filename: &str) {
//     info!("{} trying to open volume", filename);
//     let volume_future = volume_mgr.open_volume(VolumeIdx(0)).await;

//     match volume_future {
//         Ok(volume) => {
//             let root_dir = volume.open_root_dir().await.unwrap();
//             info!("\nCreating or Appending file {}...", filename);
//             let f = root_dir.open_file_in_dir(filename, Mode::ReadWriteCreateOrAppend).await.unwrap();
//             let start_timestamp: u32 = Instant::now().as_micros() as u32;
//             match f.write(buf.as_bytes()).await {
//                 Ok(_) => {
//                     let end_timestamp: u32 = Instant::now().as_micros() as u32;
//                     info!("Diff: {}", end_timestamp - start_timestamp);
//                 },

//                 Err(e) => {
//                     let end_timestamp: u32 = Instant::now().as_micros() as u32;
//                     error!("Diff: {}", end_timestamp - start_timestamp);
//                     error!("{}", defmt::Debug2Format(&e));
//                 }
//             }
            
//             f.close().await.unwrap();
//             root_dir.close().await.unwrap();
//             volume.close().await.unwrap();
//             info!("{} closed volume", filename);
//         },

//         Err(e) => {
//             error!("{}", defmt::Debug2Format(&e));
//         }
//     }

    
// }

// Writes a byte array to a file from the SD Card with full opening and closing of the volume, directory, and file for the write operation
async fn write_to_sd_card_buffer(volume_mgr: & VolumeManagerType, buf: &[u8], filename: &str) {
    let volume_future = volume_mgr.open_volume(VolumeIdx(0)).await;

    match volume_future {
        Ok(volume) => {

            let root_dir = match volume.open_root_dir().await {
                Ok(dir) => {
                    dir
                },
                Err(e) => {
                    error!("Could not open root directory! {}", Debug2Format(&e));
                    match volume.close().await {
                        Ok(_) => {

                        },
                        Err(e) => {
                            error!("Error Closing Volume: {}", Debug2Format(&e));
                        },
                    };
                    return;
                },
            };

            info!("\nCreating or Appending file {}...", filename);
            
            let f = match root_dir.open_file_in_dir(filename, Mode::ReadWriteCreateOrAppend).await {
                Ok(file) => {
                    file
                },
                Err(e) => {
                    error!("Cannot open file in root directory {}", Debug2Format(&e));
                    return;
                },
            };

            let start_timestamp: u32 = Instant::now().as_micros() as u32;
            match f.write(buf).await {
                Ok(_) => {
                    let end_timestamp: u32 = Instant::now().as_micros() as u32;
                    info!("Diff: {}", end_timestamp - start_timestamp);
                },

                Err(e) => {
                    let end_timestamp: u32 = Instant::now().as_micros() as u32;
                    error!("Diff: {}", end_timestamp - start_timestamp);
                    error!("{}", defmt::Debug2Format(&e));
                }
            }
            
            f.close().await.unwrap();
            root_dir.close().await.unwrap();
            volume.close().await.unwrap();
        },

        Err(e) => {
            error!("Couldn't open volume! {}", defmt::Debug2Format(&e));
        }
    }
}