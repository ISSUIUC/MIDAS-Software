#pragma once

// SPI sensor bus
#define SPI_MISO 3//
#define SPI_MOSI 2//
#define SPI_SCK 1//

// barometer chip select
#define MS5611_CS 44

// magnetometer chip select
#define MMC5983_CS 4
#define MMC5983_INT 7

// IMU chip select, interrupt
#define LSM_INT1 15
#define LSM_INT2 42//
#define LSM_CS 43

// i2c bus pins
#define I2C_SDA 21//
#define I2C_SCL 26//

// Buzzer pins
#define BUZZER_PIN 12
#define BUZZER_CHANNEL 1

// GPIO Expander pins
#define EXP_RESET 13
#define EXP_INT 48//

// GPS pins
#define SAM_RESET 5//

// pyro pins //hmmm
#define PYRO_GLOBAL_ARM_PIN GpioAddress(0, 3)
#define PYROA_FIRE_PIN GpioAddress(0, 0)
#define PYROB_FIRE_PIN GpioAddress(0, 1)
#define PYROC_FIRE_PIN GpioAddress(0, 7)
#define PYROD_FIRE_PIN GpioAddress(0, 6)
#define PYRO_PG GpioAddress(0, 4)

// Continuity pins (on the ADC)
#define SENSE_A 0
#define SENSE_B 1
#define SENSE_C 4
#define SENSE_D 5

// Voltage pins (on the ADC)
#define PYRO_SENSE 2 //hmmm
#define VCAP_SENSE 6
#define VBAT_SENSE 7

// E22 (radio)
#define E22_CS 37//
#define E22_DI01 41//
#define E22_DI03 40//
#define E22_BUSY 38//
#define E22_RXEN 39//
#define E22_RESET 6//

// LEDs hmmmm
#define LED_BLUE   8 
#define LED_GREEN  9 
#define LED_ORANGE 10 
#define LED_RED    11

// FLASH memory pins

#define FLASH_CMD 47//
#define FLASH_CLK 16
#define FLASH_DAT0 33//
#define FLASH_DAT1 34//
#define FLASH_DAT2 18
#define FLASH_DAT3 17

//Board to Board pins
#define B2B_EN 36
#define B2B_READY 35
