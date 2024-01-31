#pragma once

// SPI sensor bus
#define SPI_MISO 12
#define SPI_MOSI 11
#define SPI_SCK 13

// barometer chip select
#define MS5611_CS 0

// gyro chip select
#define LSM6DS3_CS 0

// high g chip select
#define KX134_CS 0

// low g chip select
#define ADXL355_CS 0

// magnetometer chip select
#define LIS3MDL_CS 0

// orientation chip select, interrupt
#define BNO086_CS 0
#define BNO086_INT 0

// voltage adc pin
#define VOLTAGE_PIN 0

// gps pins
#define GPS_RESET 0
#define GPS_ENABLE 0

// i2c bus pins
#define I2C_SDA 0
#define I2C_SCL 0

// emmc pins
#define EMMC_CLK 38
#define EMMC_CMD 39
#define EMMC_D0 44
#define EMMC_D1 43
#define EMMC_D2 2
#define EMMC_D3 42