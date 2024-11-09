#include <Wire.h>
#include <SPI.h>
#include "TCAL9539.h"

#include "systems.h"
#include "hardware/pins.h"
#include "hardware/Emmc.h"
#include "hardware/SDLog.h"
#include "sensor_data.h"


#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "BLEmyCharacteristicCallbacks.h"


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

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

void setup() {
  Serial.begin(9600);
  Serial.println("Starting BLE work!");
  while(!Serial);

  BLEmyCharacteristicCallbacks pmyCallback = BLEmyCharacteristicCallbacks();

    //begin sensor SPI bus

    // Serial.println("Starting SPI...");
    // SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    // //begin I2C bus
    // Serial.println("Starting I2C...");
    // Wire.begin(I2C_SDA, I2C_SCL);

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

    // gpioPinMode(LED_BLUE, OUTPUT);
    // gpioPinMode(LED_GREEN, OUTPUT);
    // gpioPinMode(LED_ORANGE, OUTPUT);
    // gpioPinMode(LED_RED, OUTPUT);

  BLEDevice::init("MyESP32");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(&pmyCallback);
  pCharacteristic->setValue("Hello World says Neil");
  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");



    // begin_systems(&systems);

//     //begin serial port
//     Serial.begin(9600);

// //    while (!Serial);

//     delay(200);

//     //begin sensor SPI bus
//     Serial.println("Starting SPI...");
//     SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

//     //begin I2C bus
//     Serial.println("Starting I2C...");
//     Wire.begin(I2C_SDA, I2C_SCL);

//     //set all chip selects high (deselected)
//     pinMode(MS5611_CS, OUTPUT);
//     pinMode(LSM6DS3_CS, OUTPUT);
//     pinMode(KX134_CS, OUTPUT);
//     pinMode(ADXL355_CS, OUTPUT);
//     pinMode(LIS3MDL_CS, OUTPUT);
//     pinMode(BNO086_CS, OUTPUT);
//     pinMode(CAN_CS, OUTPUT);
//     pinMode(RFM96_CS, OUTPUT);
//     digitalWrite(MS5611_CS, HIGH);
//     digitalWrite(LSM6DS3_CS, HIGH);
//     digitalWrite(KX134_CS, HIGH);
//     digitalWrite(ADXL355_CS, HIGH);
//     digitalWrite(LIS3MDL_CS, HIGH);
//     digitalWrite(BNO086_CS, HIGH);
//     digitalWrite(CAN_CS, HIGH);
//     digitalWrite(RFM96_CS, HIGH);

//     //configure output leds
//     gpioPinMode(LED_BLUE, OUTPUT);
//     gpioPinMode(LED_GREEN, OUTPUT);
//     gpioPinMode(LED_ORANGE, OUTPUT);
//     gpioPinMode(LED_RED, OUTPUT);

//     delay(200);

//     //init and start threads
//     begin_systems(&systems);
}

void loop() {
    delay(2000);
    //Serial.print("hello");
}
