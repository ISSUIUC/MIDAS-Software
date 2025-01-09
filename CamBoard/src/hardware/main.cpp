#include <Arduino.h>

#include <Wire.h>
#include <SPI.h>
#include <HardwareSerial.h>

#include "hardware/pins.h"

/**
 * Sets the config file and then starts all the threads using the config.
 */


// RocketSystems systems { .log_sink = sinks };
// /**
//  * @brief Sets up pinmodes for all sensors and starts threads
// */

void setup() {
    //begin serial port
    Serial.begin(9600);

//    while (!Serial);

    delay(200);

    // //begin sensor SPI bus
    // Serial.println("Starting SPI...");
    // SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    // //begin I2C bus
    // Serial.println("Starting I2C...");
    // Wire.begin(I2C_SDA, I2C_SCL);

    // //set all chip selects high (deselected)
    // pinMode(MS5611_CS, OUTPUT);
    // pinMode(LSM6DS3_CS, OUTPUT);
    // pinMode(KX134_CS, OUTPUT);
    // pinMode(ADXL355_CS, OUTPUT);
    // pinMode(LIS3MDL_CS, OUTPUT);
    // pinMode(BNO086_CS, OUTPUT);
    // pinMode(CAN_CS, OUTPUT);
    // pinMode(RFM96_CS, OUTPUT);
    // digitalWrite(MS5611_CS, HIGH);
    // digitalWrite(LSM6DS3_CS, HIGH);
    // digitalWrite(KX134_CS, HIGH);
    // digitalWrite(ADXL355_CS, HIGH);
    // digitalWrite(LIS3MDL_CS, HIGH);
    // digitalWrite(BNO086_CS, HIGH);
    // digitalWrite(CAN_CS, HIGH);
    // digitalWrite(RFM96_CS, HIGH);

    // //configure output leds
    // gpioPinMode(LED_BLUE, OUTPUT);
    // gpioPinMode(LED_GREEN, OUTPUT);
    // gpioPinMode(LED_ORANGE, OUTPUT);
    // gpioPinMode(LED_RED, OUTPUT);

    // delay(200);

    // //init and start threads
    // begin_systems(&systems);
}

void loop() {

}

// #define LED_TEST
// // #define REGULATOR_TEST

// #define BLUE_LED_PIN 37
// #define REGULATOR_PIN 9

// void setup() {
//   // put your setup code here, to run once:
//   // Serial.begin(9600);
//   // while(!Serial);

//   // Serial.println("Beginning cam board test setup");
//   #ifdef LED_TEST
//     pinMode(BLUE_LED_PIN, OUTPUT); // Set BLUE LED to output
//     // Serial.println("Set LED pinmode");
//   #endif

//   #ifdef REGULATOR_TEST
//     pinMode(REGULATOR_PIN, OUTPUT); // Set 9V regulator to output
//     Serial.println("Set Regulator pinmode");
//   #endif

//   // Serial.println("cam board test setup complete\n");
// }

// void loop() {
//   // put your main code here, to run repeatedly:

//   #ifdef REGULATOR_TEST
//     Serial.println("Regulator set high");
//     digitalWrite(REGULATOR_PIN, HIGH);
//   #endif

//   #ifdef LED_TEST
//     // Flash blue LED at 1hz (ish)
//     // Serial.println("LED test Loop high");
//     digitalWrite(BLUE_LED_PIN, HIGH);
//     delay(500);
//     // Serial.println("LED test Loop low");
//     digitalWrite(BLUE_LED_PIN, LOW);
//     delay(500);
//   #endif

//   delay(10);
// }