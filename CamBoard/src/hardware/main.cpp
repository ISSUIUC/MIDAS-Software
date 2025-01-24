#include <Arduino.h>

#include <Wire.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include <ACAN2517FD.h>
#include <ACAN2517FDSettings.h>

#include "hardware/pins.h"
#include "systems.h"

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

    //begin sensor SPI bus
    Serial.println("Starting SPI...");
    SPI.begin(CAN_SPI_SCK, CAN_SPI_MISO, CAN_SPI_MOSI);

    //begin I2C bus
        // Serial.println("Starting I2C...");
        // Wire.begin(I2C_SDA, I2C_SCL);
    Serial.println("Starting Battery Sense I2C...");
    Wire.begin(BATTSENSE_SDA, BATTSENSE_SCL);

    //begin UART
    Serial.println("Starting UART...");
    HardwareSerial CAM_1_UART(1);
    CAM_1_UART.begin(115200, SERIAL_8N1, CAM1_RX, CAM1_TX);
    HardwareSerial CAM_2_UART(2);
    CAM_2_UART.begin(115200, SERIAL_8N1, CAM2_RX, CAM2_TX);

    //begin Camera Control
    Serial.println("Starting Camera Control...");
    pinMode(CAM1_ON_OFF, OUTPUT);
    digitalWrite(CAM1_ON_OFF, LOW);
    pinMode(CAM2_ON_OFF, OUTPUT);
    digitalWrite(CAM2_ON_OFF, LOW);
    pinMode(VTX_ON_OFF, OUTPUT);
    digitalWrite(VTX_ON_OFF, LOW);
    pinMode(VIDEO_SELECT, OUTPUT);
    digitalWrite(VIDEO_SELECT, LOW);

    //begin CAN
    Serial.println("Starting CAN...");
    pinMode(CAN_NINT, INPUT);
    pinMode(CAN_NINT1, INPUT);
    pinMode(CAN_SLNT, OUTPUT);
    pinMode(CAN_FAULT, INPUT);
    pinMode(CAN_NCS, OUTPUT);
    digitalWrite(CAN_NCS, LOW);  //selected
    digitalWrite(CAN_SLNT, LOW);

    pinMode(BATTSENSE_ALERT, INPUT);
    //pinMode(BUZZER, OUTPUT);




    // //set all chip selects high (deselected)
    // pinMode(MS5611_CS, OUTPUT);
    // pinMode(LSM6DS3_CS, OUTPUT);
    // pinMode(KX134_CS, OUTPUT);
    // pinMode(ADXL355_CS, OUTPUT);
    // pinMode(LIS3MDL_CS, OUTPUT);
    // pinMode(BNO086_CS, OUTPUT);
    // pinMode(RFM96_CS, OUTPUT);
    // digitalWrite(MS5611_CS, HIGH);
    // digitalWrite(LSM6DS3_CS, HIGH);
    // digitalWrite(KX134_CS, HIGH);
    // digitalWrite(ADXL355_CS, HIGH);
    // digitalWrite(LIS3MDL_CS, HIGH);
    // digitalWrite(BNO086_CS, HIGH);
    // digitalWrite(CAN_CS, HIGH);
    // digitalWrite(RFM96_CS, HIGH);

    //configure output leds
    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_ORANGE, OUTPUT);
    pinMode(LED_RED, OUTPUT);

    delay(200);

    //init and start threads
    begin_systems(&systems);
}

void loop() {

}

// #define LED_TEST

// void setup() {
//   // put your setup code here, to run once:
//   // Serial.begin(9600);
//   // while(!Serial);

//   // Serial.println("Beginning cam board test setup");
//   #ifdef LED_TEST
//     pinMode(BLUE_LED_PIN, OUTPUT); // Set BLUE LED to output
//     // Serial.println("Set LED pinmode");
//   #endif


//   // Serial.println("cam board test setup complete\n");
// }

// void loop() {
//   // put your main code here, to run repeatedly:

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