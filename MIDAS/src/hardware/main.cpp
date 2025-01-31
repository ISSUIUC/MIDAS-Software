#include <Wire.h>
#include <SPI.h>
#include "TCAL9539.h"

#include "systems.h"
#include "hardware/pins.h"
#include "hardware/Emmc.h"
#include "SDLog.h"
#include "sensor_data.h"

/**
 * Sets the config file and then starts all the threads using the config.
 */

#ifdef IS_SUSTAINER
// MultipleLogSink<EMMCSink> sinks;
MultipleLogSink<SDSink> sinks;
#else
MultipleLogSink<> sinks;
#endif
RocketSystems systems { .log_sink = sinks };
/**
 * @brief Sets up pinmodes for all sensors and starts threads
*/

void setup() {
    //begin serial port
    Serial.begin(9600);

//    while (!Serial);

    delay(200);

    //begin sensor SPI bus
    Serial.println("Starting SPI...");
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    //begin I2C bus
    Serial.println("Starting I2C...");
    Wire.begin(I2C_SDA, I2C_SCL);

    //set all chip selects high (deselected)
    pinMode(MS5611_CS, OUTPUT);
    pinMode(LSM6DS3_CS, OUTPUT);
    pinMode(KX134_CS, OUTPUT);
    pinMode(ADXL355_CS, OUTPUT);
    pinMode(LIS3MDL_CS, OUTPUT);
    pinMode(BNO086_CS, OUTPUT);
    pinMode(CAN_CS, OUTPUT);
    pinMode(RFM96_CS, OUTPUT);
    digitalWrite(MS5611_CS, HIGH);
    digitalWrite(LSM6DS3_CS, HIGH);
    digitalWrite(KX134_CS, HIGH);
    digitalWrite(ADXL355_CS, HIGH);
    digitalWrite(LIS3MDL_CS, HIGH);
    digitalWrite(BNO086_CS, HIGH);
    digitalWrite(CAN_CS, HIGH);
    digitalWrite(RFM96_CS, HIGH);

    //configure output leds
    gpioPinMode(LED_BLUE, OUTPUT);
    gpioPinMode(LED_GREEN, OUTPUT);
    gpioPinMode(LED_ORANGE, OUTPUT);
    gpioPinMode(LED_RED, OUTPUT);

    delay(200);

    //init and start threads
    begin_systems(&systems);
}

void loop() {

}
