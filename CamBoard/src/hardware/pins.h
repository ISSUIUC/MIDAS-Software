#pragma once

// // SPI sensor bus
// #define SPI_MISO 13
// #define SPI_MOSI 11
// #define SPI_SCK 12

// // barometer chip select
// #define MS5611_CS 14

// // gyro chip select
// #define LSM6DS3_CS 3

// // high g chip select
// #define KX134_CS 10

// // low g chip select
// #define ADXL355_CS 0

// // magnetometer chip select
// #define LIS3MDL_CS 9

// // orientation chip select, interrupt
// #define BNO086_CS 21
// #define BNO086_INT 47
// #define BNO086_RESET GpioAddress(1, 07)

// // voltage adc pin
// #define VOLTAGE_PIN 0

// // gps pins
// #define GNSS_I2C_LOCATION 0x3A
// #define GPS_RESET GpioAddress(2, 017)
// #define GPS_ENABLE 0

// // i2c bus pins
// #define I2C_SDA 18
// #define I2C_SCL 8

// // can pin
// #define CAN_CS 45

// // emmc pins
// #define EMMC_CLK 38
// #define EMMC_CMD 39
// #define EMMC_D0 44
// #define EMMC_D1 43
// #define EMMC_D2 2
// #define EMMC_D3 42

// // SD Pin(s)
// #define SD_CLK 5
// #define SD_CMD 4
// #define SD_D0 6

// // pyro pins
// #define PYRO_GLOBAL_ARM_PIN GpioAddress(0, 07)
// #define PYROA_ARM_PIN GpioAddress(0, 016)
// #define PYROA_FIRE_PIN GpioAddress(0, 017)
// #define PYROB_ARM_PIN GpioAddress(0, 014)
// #define PYROB_FIRE_PIN GpioAddress(0, 015)
// #define PYROC_ARM_PIN GpioAddress(0, 010)
// #define PYROC_FIRE_PIN GpioAddress(0, 011)
// #define PYROD_ARM_PIN GpioAddress(0, 012)
// #define PYROD_FIRE_PIN GpioAddress(0, 013)

// // Continuity Pins
// #define SENSE_PYRO 1
// #define SENSE_APOGEE 6
// #define SENSE_MAIN 7
// #define SENSE_MOTOR 4
// #define SENSE_AUX 5

// // Telemetry pins
// #define RFM96_CS 1
// #define RFM96_INT 7
// #define RFM96_RESET 15

// // LEDs
// #define LED_BLUE   GpioAddress(2, 013)
// #define LED_GREEN  GpioAddress(2, 014)
// #define LED_ORANGE GpioAddress(2, 015)
// #define LED_RED    GpioAddress(2, 016)

// Camera/Video Control Pins
#define CAM1_ON_OFF 3
#define CAM2_ON_OFF 2
#define VTX_ON_OFF 4

#define VIDEO_SELECT 1

#define CAM1_TX 8
#define CAM1_RX 9
#define CAM2_TX 10
#define CAM2_RX 11

// I2C Busses
#define BATTSENSE_SCL 5
#define BATTSENSE_SDA 6
#define BATTSENSE_ALERT 7

#define I2C_SCL 36
#define I2C_SDA 37

// CAN
#define CAN_SPI_SCK 14
#define CAN_SPI_MOSI 15
#define CAN_SPI_MISO 16
#define CAN_NCS 17
#define CAN_NINT 12
#define CAN_NINT1 13
#define CAN_SLNT 21
#define CAN_FAULT 26

// Buzzer - defined in buzzer.cpp
// #define BUZZER 35

//LEDs
#define LED_BLUE 39
#define LED_GREEN 40
#define LED_ORANGE 41
#define LED_RED 42