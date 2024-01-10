#include "systems.h"
#include "hardware/pins.h"
#include <SPI.h>
#include <Wire.h>

/**
 * Sets the config file and then starts all the threads using the config.
 */
RocketSystems systems;

void setup() {
    //begin serial port
    Serial.begin(9600);

    //begin sensor SPI bus
    Serial.println("Starting SPI...");
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    //begin I2C bus
    Serial.println("Starting I2C...");
    Wire.begin(I2C_SDA, I2C_SCL);

    begin_systems(&systems);
}

void loop() {

}
