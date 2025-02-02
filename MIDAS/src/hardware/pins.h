#pragma once

// SPI sensor bus
#define SPI_MISO 39
#define SPI_MOSI 37
#define SPI_SCK 38

// barometer chip select
#define MS5611_CS 43

// gyro chip select
#define LSM6DS3_CS 36

// high g chip select
#define KX134_CS 40

// low g chip select
#define ADXL355_CS 44

// magnetometer chip select
#define LIS3MDL_CS 35

// orientation chip select, interrupt
#define BNO086_CS 48
#define BNO086_INT 34
#define BNO086_RESET 33

// voltage adc pin
#define VOLTAGE_PIN 0

// gps pins
#define GNSS_I2C_LOCATION 0x3A
#define GPS_RESET GpioAddress(2, 017)
#define GPS_ENABLE 0

// i2c bus pins
#define I2C_SDA 10
#define I2C_SCL 9

#define PYRO_SDA 41
#define PYRO_SCL 42

// CAN pins
#define CAN_CS 2
#define RFM96W_CS 5

// GPS I2C location
#define GNSS_I2C_LOCATION 0x3A
#define GPS_RESET GpioAddress(2, 017)
#define GPS_ENABLE 0

//take the pins that are in base 8 and update it to the right base - this is not done yet.

// pyro pins
#define PYRO_GLOBAL_ARM_PIN GpioAddress(0, 05)
#define PYROA_ARM_PIN GpioAddress(0, 016)
#define PYROA_FIRE_PIN GpioAddress(0, 04)
#define PYROB_ARM_PIN GpioAddress(0, 014)
#define PYROB_FIRE_PIN GpioAddress(0, 03)
#define PYROC_ARM_PIN GpioAddress(0, 010)
#define PYROC_FIRE_PIN GpioAddress(0, 01)
#define PYROD_ARM_PIN GpioAddress(0, 0)
#define PYROD_FIRE_PIN GpioAddress(0, 00)

// Continuity Pins
#define SENSE_APOGEE 010
#define SENSE_MAIN 011
#define SENSE_MOTOR 06
#define SENSE_AUX 07

// Telemetry pins
#define RFM96_INT 7 // DEPRECTED NO LONGER USE

// E22
#define E22_CS 5
#define E22_DI01 4
#define E22_DI03 3
#define E22_BUSY 6
#define E22_RXEN 7
#define E22_RESET 8

// LEDs
#define LED_BLUE   GpioAddress(2, 014)
#define LED_GREEN  GpioAddress(2, 015)
#define LED_ORANGE GpioAddress(2, 016)
#define LED_RED    GpioAddress(2, 017)

#define FLASH_CMD 14
#define FLASH_CLK 18
#define FLASH_DAT0 13
#define FLASH_DAT1 12
#define FLASH_DAT2 15
#define FLASH_DAT3 16