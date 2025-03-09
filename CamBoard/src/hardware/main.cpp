#include <Arduino.h>

#include <Wire.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include <ACAN2517FD.h>
#include <ACAN2517FDSettings.h>

#include "hardware/pins.h"
#include "systems.h"
#include "camera.h"

#define CAMBOARD_I2C_ADDR 0x69



enum class CameraCommand {
    CAMERA0_OFF = 0,
    CAMERA0_ON = 1,
    CAMERA1_OFF = 2,
    CAMERA1_ON = 3,
    VTX_OFF = 4,
    VTX_ON = 5,
    MUX_0 = 6,
    MUX_1 = 7
  };

void onReceive(int len) {
    Serial.print("Recieved: ");
    while (Wire1.available()) {
      uint8_t recieve = Wire1.read();
      Serial.print(recieve);
      Serial.print(": ");
      
      switch(recieve) {
        case 0:{
          Serial.println("Case 0\n");

          int change = 1;
          while(change == 1) {
            Serial.println("Trying to turn off camera 1");
            
            struct read_mem_cap_data_return toReturn1;
            toReturn1 = read_mem_cap_data(Serial1);
            while(toReturn1.status == 0) {
              toReturn1 = read_mem_cap_data(Serial1);
            }

            struct read_mem_cap_data_return toReturn2;
            toReturn2 = read_mem_cap_data(Serial1);
            while(toReturn2.status == 0) {
              toReturn2 = read_mem_cap_data(Serial1);
            }

            int i;
            for(i = 0; i < 12; i++) {
              if(toReturn1.buf[i] != toReturn2.buf[2]) {
                change = 1;
                break;
              }
            }

            if(i == 12) {
              change = 0;
            }

            if(change == 1) {
              camera_on_off(Serial1);
            }

            delay(5000);
          }

          digitalWrite(CAM1_ON_OFF, LOW);

          break;}
        case 1: {
          Serial.println("Case 1\n");

          int change = 0;

          while(change == 0) {
            Serial.println("Trying to turn on camera 1");
            digitalWrite(CAM1_ON_OFF, HIGH);
            
            struct read_mem_cap_data_return toReturn1;
            toReturn1 = read_mem_cap_data(Serial1);
            while(toReturn1.status == 0) {
              toReturn1 = read_mem_cap_data(Serial1);
            }

            struct read_mem_cap_data_return toReturn2;
            toReturn2 = read_mem_cap_data(Serial1);
            while(toReturn2.status == 0) {
              toReturn2 = read_mem_cap_data(Serial1);
            }

            for(int i = 0; i < 12; i++) {
              if(toReturn1.buf[i] != toReturn2.buf[2]) {
                change = 1;
                break;
              }
            }

            if(change == 0) {
              camera_on_off(Serial1);
            }
            delay(5000);
          }
          
          break;}
        case 2:{
          Serial.println("Case 2\n");

          int change = 1;
          while(change == 1) {
            Serial.println("Trying to turn off camera 2");
            
            struct read_mem_cap_data_return toReturn1;
            toReturn1 = read_mem_cap_data(Serial2);
            while(toReturn1.status == 0) {
              toReturn1 = read_mem_cap_data(Serial2);
            }

            struct read_mem_cap_data_return toReturn2;
            toReturn2 = read_mem_cap_data(Serial2);
            while(toReturn2.status == 0) {
              toReturn2 = read_mem_cap_data(Serial2);
            }

            int i;
            for(i = 0; i < 12; i++) {
              if(toReturn1.buf[i] != toReturn2.buf[2]) {
                change = 1;
                break;
              }
            }

            if(i == 12) {
              change = 0;
            }

            if(change == 1) {
              camera_on_off(Serial2);
            }
            
            delay(5000);
          }

          digitalWrite(CAM2_ON_OFF, LOW);
          break;}
        case 3: {
          Serial.println("Case 3\n");

          int change = 0;
          while(change == 0) {
            Serial.println("Trying to turn on camera 2");
            digitalWrite(CAM2_ON_OFF, HIGH);
            
            struct read_mem_cap_data_return toReturn1;
            toReturn1 = read_mem_cap_data(Serial2);
            while(toReturn1.status == 0) {
              toReturn1 = read_mem_cap_data(Serial2);
            }

            struct read_mem_cap_data_return toReturn2;
            toReturn2 = read_mem_cap_data(Serial2);
            while(toReturn2.status == 0) {
              toReturn2 = read_mem_cap_data(Serial2);
            }

            for(int i = 0; i < 12; i++) {
              if(toReturn1.buf[i] != toReturn2.buf[2]) {
                change = 1;
                break;
              }
            }

            if(change == 0) {
              camera_on_off(Serial2);
            }
            delay(5000);
          }
          break;}
        case 4:
          digitalWrite(VTX_ON_OFF, LOW);
          Serial.println("Case 4\n");
          break;
        case 5:
          digitalWrite(VTX_ON_OFF, HIGH);
          Serial.println("Case 5\n");
          break;
        case 6:
          digitalWrite(VIDEO_SELECT, LOW);
          Serial.println("Case 6\n");
          break;
        case 7:
          digitalWrite(VIDEO_SELECT, HIGH);
          Serial.println("Case 7\n");
          break;
        default:
          break;
      }
    }
    Serial.println("(EOT)");
  }


/**
 * Sets the config file and then starts all the threads using the config.
 */
// HardwareSerial (1);
// HardwareSerial cam2(2);

//RocketSystems systems{RocketData{}, BuzzerController{}, LEDController{}, Cameras{cam1, cam2}, CAN{}};
// /**
//  * @brief Sets up pinmodes for all sensors and starts threads
// */
void setup() {
    //begin serial port
    Serial.begin(9600);


    delay(5000);

    Serial1.setPins(CAM1_RX, CAM1_TX);
    Serial2.setPins(CAM2_RX, CAM2_TX);
    RocketSystems systems{RocketData{}, BuzzerController{}, LEDController{}, Cameras{&Serial1, &Serial2}, CAN{}};


    //begin sensor SPI bus
    Serial.println("Starting SPI...");
    SPI.begin(CAN_SPI_SCK, CAN_SPI_MISO, CAN_SPI_MOSI);

    //begin I2C bus
        // Serial.println("Starting I2C...");
        // Wire.begin(I2C_SDA, I2C_SCL);
    Serial.println("Starting Battery Sense I2C...");
    Wire.begin(BATTSENSE_SDA, BATTSENSE_SCL);
    Wire1.begin(I2C_SDA, I2C_SCL);
    Wire1.onReceive(onReceive);
    Wire1.begin((uint8_t)CAMBOARD_I2C_ADDR);

    //begin UART
    // Serial.println("Starting UART...");
    // HardwareSerial CAM_1_UART(1);
    // CAM_1_UART.begin(115200, SERIAL_8N1, CAM1_RX, CAM1_TX);
    // HardwareSerial CAM_2_UART(2);
    // CAM_2_UART.begin(115200, SERIAL_8N1, CAM2_RX, CAM2_TX);

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

    //digitalWrite(CAM1_ON_OFF, HIGH);


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

    //configure output leds
    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_ORANGE, OUTPUT);
    pinMode(LED_RED, OUTPUT);

    delay(200);

    //init and start threads
    begin_systems(&systems);

    // Serial1.begin(115200, SERIAL_8N1, CAM1_RX, CAM1_TX);
    // digitalWrite(CAM1_ON_OFF, HIGH);
    // camera_on_off(Serial1);
    // start_recording(Serial1);
    // Serial.println("Now recording");


    //Camera test code

    //cam1.begin(115200, SERIAL_8N1, CAM1_RX, CAM1_TX);
    //cam2.begin(115200, SERIAL_8N1, CAM2_RX, CAM2_TX);
    // digitalWrite(CAM1_ON_OFF, HIGH);
    // digitalWrite(CAM2_ON_OFF, HIGH);
    // digitalWrite(VTX_ON_OFF, HIGH);

    // camera_on_off(cam1);
    // camera_on_off(cam2);
    // start_recording(cam1);
    // start_recording(cam2);
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