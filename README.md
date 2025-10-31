Special Package Requirements:

The BME280 package uses a feature of the Rust language that is unstable, so I had to change some settings to make the default setting to be async in the BME280 package because otherwise the unstable feature would still make the package think that I was using the sync version instead of the async version. This also means you have to run this project on the unstable toolchain of rust. Unstable Issues on Github: https://github.com/rust-lang/rust/issues/63063 and https://github.com/rust-lang/rust/pull/110237. Here is the Github Page: https://github.com/wisewhyforge/bme280-rs-async-default

There also needs to be a custom embedded sdmmc async file you would have to import and store in the path specified in the cargo.toml file. For example: 
embedded-sdmmc-async = {path = "../embedded-sdmmc-rs-async-attempt"} means that the path needs to be stored in the previous folder and than this one with the name: embedded-sdmmc-rs-async-attempt. Here is the repo to the specific cargo package since my package is not on the cargo registry. https://github.com/wisewhyforge/embedded-sdmmc-rs-async-attempt

The ADXL345 Async Package needs to be put in the same folder where embedded-sdmmc-async is stored. Here is the link to the repository: https://gitlab.com/wisewhyforge-group/ADXL345-Async-Rs#