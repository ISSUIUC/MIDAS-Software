#include <Arduino.h>

#include <Wire.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include <ACAN2517FD.h>
#include <ACAN2517FDSettings.h>
#include <EEPROM.h>

#include "hardware/pins.h"
#include "systems.h"
#include "camera.h"

#define EEPROM_SIZE 64

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

extern cam_state_t GLOBAL_CAM_STATE;
extern cam_state_t DESIRED_CAM_STATE;
extern uint32_t LAST_I2C_COMM;

bool BLUE_LED_STATE = false;
bool GREEN_LED_STATE = false;

void onRequest() {
  uint8_t cam_dat = 0;
  // encode data

  // "FSM" thread data
  cam_dat |= (GLOBAL_CAM_STATE.cam1_on) << 1;
  cam_dat |= (GLOBAL_CAM_STATE.cam2_on) << 0;
  cam_dat |= (GLOBAL_CAM_STATE.cam1_rec) << 3;
  cam_dat |= (GLOBAL_CAM_STATE.cam2_rec) << 2;

  // Main thread data
  cam_dat |= (GLOBAL_CAM_STATE.vtx_on) << 4;
  cam_dat |= (GLOBAL_CAM_STATE.vmux_state) << 5;
  cam_dat |= (GLOBAL_CAM_STATE.cam_ack) << 6;

  // Toggle blue LED
  BLUE_LED_STATE = !BLUE_LED_STATE;
  digitalWrite(LED_BLUE, BLUE_LED_STATE ? HIGH : LOW);

  // print for debug
  Serial.print("telemetry: CAM 1 (on / recording) ");
  Serial.print(GLOBAL_CAM_STATE.cam1_on);
  Serial.print(" / ");
  Serial.print(GLOBAL_CAM_STATE.cam1_rec);
  Serial.print("    CAM 2 (on / recording) ");
  Serial.print(GLOBAL_CAM_STATE.cam2_on);
  Serial.print(" / ");
  Serial.println(GLOBAL_CAM_STATE.cam2_rec);

  // Send to MIDAS

  uint8_t buf[1] = { cam_dat };
  LAST_I2C_COMM = millis();
  Wire1.slaveWrite(buf, 1);
}

void onReceive(int len) {
    Serial.print("Recieved: ");
    while (Wire1.available()) {
      uint8_t recieve = Wire1.read();
      Serial.print(recieve);
      Serial.print(": ");

      GREEN_LED_STATE = !GREEN_LED_STATE;
      digitalWrite(LED_GREEN, GREEN_LED_STATE ? HIGH : LOW);

      GLOBAL_CAM_STATE.cam_ack = !GLOBAL_CAM_STATE.cam_ack;
      switch(recieve) {
        case 0: {
          Serial.println("Case 0\n");
          camera_on_off(Serial1); // Stop recording
          DESIRED_CAM_STATE.cam1_on = false; // Attempt to turn off the camera when recording stopped.
          break;}
        case 1: {
          Serial.println("Case 1\n");
          DESIRED_CAM_STATE.cam1_on = true;
          Serial.println("Trying to turn on camera 1");
          digitalWrite(CAM1_ON_OFF, HIGH);
          break;}
        case 2:{
          Serial.println("Case 2\n");
          camera_on_off(Serial2); // Stop recording
          DESIRED_CAM_STATE.cam2_on = false; // Attempt to turn off the camera when recording stopped.
          break;}
        case 3: {
          Serial.println("Case 3\n");
          DESIRED_CAM_STATE.cam2_on = true;
          Serial.println("Trying to turn on camera 2");
          digitalWrite(CAM2_ON_OFF, HIGH);
          break;}
        case 4:
          digitalWrite(VTX_ON_OFF, LOW);
          Serial.println("Case 4\n");
          GLOBAL_CAM_STATE.vtx_on = false;
          DESIRED_CAM_STATE.vtx_on = false;
          break;
        case 5:
          digitalWrite(VTX_ON_OFF, HIGH);
          Serial.println("Case 5\n");
          GLOBAL_CAM_STATE.vtx_on = true;
          DESIRED_CAM_STATE.vtx_on = true;
          break;
        case 6:
          digitalWrite(VIDEO_SELECT, LOW);
          Serial.println("Case 6\n");
          GLOBAL_CAM_STATE.vmux_state = false;
          break;
        case 7:
          digitalWrite(VIDEO_SELECT, HIGH);
          Serial.println("Case 7\n");
          GLOBAL_CAM_STATE.vmux_state = true;
          break;
        default:
          break;
      }
    }
    Serial.println("(EOT)");
  }


  
void update_desired_state(uint8_t state_byte) {
  DESIRED_CAM_STATE.cam1_on = state_byte & 0b00000001;
  DESIRED_CAM_STATE.cam2_on = state_byte & 0b00000010;
  DESIRED_CAM_STATE.vtx_on = state_byte & 0b00000100;
  DESIRED_CAM_STATE.vmux_state = state_byte & 0b00001000;
  DESIRED_CAM_STATE.cam1_rec = state_byte & 0b00010000;
  DESIRED_CAM_STATE.cam2_rec = state_byte & 0b00100000;

  // Turn on cameras if we want them to be on
  digitalWrite(CAM1_ON_OFF, DESIRED_CAM_STATE.cam1_on ? HIGH : LOW);
  digitalWrite(CAM2_ON_OFF, DESIRED_CAM_STATE.cam2_on ? HIGH : LOW);
  digitalWrite(VTX_ON_OFF, DESIRED_CAM_STATE.vtx_on ? HIGH : LOW);
  digitalWrite(VIDEO_SELECT, DESIRED_CAM_STATE.vmux_state ? HIGH : LOW);
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

    // Immediate buzzer tone
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);

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
    Wire1.setPins(I2C_SDA, I2C_SCL);
    Wire1.onReceive(onReceive);
    Wire1.onRequest(onRequest);
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

    ledcWriteTone(BUZZER_CHANNEL, 2730);
    delay(300);
    ledcWriteTone(BUZZER_CHANNEL, 0);

    // Read the desired state from flash memory
    EEPROM.begin((size_t)EEPROM_SIZE);

    // EEPROM.write(0, 0); // For resetting flash
    // EEPROM.commit();
    // delay(999999);

    uint8_t desired_state = EEPROM.read(0);
    Serial.println(desired_state, 2);
    update_desired_state(desired_state);

    // Signs of life

    delay(500);
    ledcWriteTone(BUZZER_CHANNEL, 2730);
    delay(300);
    ledcWriteTone(BUZZER_CHANNEL, 0);

    delay(100);

    // CAM state will be given by 3 beeps -- CAM1, CAM2, VTX. HIGH means it's on, LOW means it's off.
    if(DESIRED_CAM_STATE.cam1_on) {
      ledcWriteTone(BUZZER_CHANNEL, 2800);
      delay(120);
      ledcWriteTone(BUZZER_CHANNEL, 0);
    } else {
      ledcWriteTone(BUZZER_CHANNEL, 2300);
      delay(120);
      ledcWriteTone(BUZZER_CHANNEL, 0);
    }
    delay(40);

    if(DESIRED_CAM_STATE.cam2_on) {
      ledcWriteTone(BUZZER_CHANNEL, 2800);
      delay(120);
      ledcWriteTone(BUZZER_CHANNEL, 0);
    } else {
      ledcWriteTone(BUZZER_CHANNEL, 2300);
      delay(120);
      ledcWriteTone(BUZZER_CHANNEL, 0);
    }
    delay(40);

    if(DESIRED_CAM_STATE.vtx_on) {
      ledcWriteTone(BUZZER_CHANNEL, 2800);
      delay(120);
      ledcWriteTone(BUZZER_CHANNEL, 0);
    } else {
      ledcWriteTone(BUZZER_CHANNEL, 2300);
      delay(120);
      ledcWriteTone(BUZZER_CHANNEL, 0);
    }
    delay(140);

    //init and start threads
    LAST_I2C_COMM = millis();
    begin_systems(&systems);
}

void loop() {

}
