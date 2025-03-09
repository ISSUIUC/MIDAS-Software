#include <HardwareSerial.h>
#include "camera.h"
#include "pins.h"

//https://support.runcam.com/hc/en-us/articles/360014537794-RunCam-Device-Protocol

ErrorCode Cameras::init() {
    Serial.println("Starting UART...");
    //HardwareSerial cam1(1);
    cam1->begin(115200, SERIAL_8N1, CAM1_RX, CAM1_TX);
    //HardwareSerial cam2(2);
    //cam2->begin(115200, SERIAL_8N1, CAM2_RX, CAM2_TX);

    digitalWrite(CAM1_ON_OFF, HIGH);
    //digitalWrite(CAM2_ON_OFF, HIGH);
    //digitalWrite(VTX_ON_OFF, HIGH);
   Serial.println("Finished setting up cameras");
    return ErrorCode::NoError; // Change this
}

void camera_on_off(HardwareSerial& camera) {
    Serial.println("Turning camera on");
    uint8_t arr[4] = {0xCC, 0x01, 0x01, 0xE7};
        camera.write(arr, 4);
        camera.flush();
    Serial.println("Finished turning camera on");
    Serial.flush();
}

void start_recording(HardwareSerial& camera) {
    Serial.println("Starting camera recording");
    uint8_t arr[4] = {0xCC, 0x01, 0x03, 0x98};
        camera.write(arr, 4);
        camera.flush();
    Serial.println("Finished camera recording");
    Serial.flush();
}

void stop_recording(HardwareSerial& camera) {
    Serial.println("Stopping camera recording");
    uint8_t arr[4] = {0xCC, 0x01, 0x04, 0xCC};
        camera.write(arr, 4);
        camera.flush();
    Serial.println("Finished camera recording");
    Serial.flush();
}

uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a) {
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }

    return crc;
}

uint8_t generate_crc(uint8_t* buf, unsigned int buf_len) {
  uint8_t crc = 0x00;
  for(unsigned i = 0; i < buf_len; i++) {
    crc = crc8_dvb_s2(crc, buf[i]);
  }
  return crc;
}

bool check_crc(uint8_t* buf, unsigned int buf_len, uint8_t expected_crc) {
  return generate_crc(buf, buf_len) == expected_crc;
}

// static unsigned gSendDate = 0 ;
// static unsigned gSentCount = 0 ;

struct read_mem_cap_data_return read_mem_cap_data(HardwareSerial& camera) {
  uint8_t get_setting_raw[4] = {0xCC, 0x11, 0x03, 0x00};
  uint8_t get_setting[5] = {0xCC, 0x11, 0x03, 0x00, generate_crc(get_setting_raw, 4)};
  Serial1.write(get_setting, 5);
  Serial.write("Reading...");
  delay(2000);
  
  struct read_mem_cap_data_return toReturn;
  toReturn.status = 0;

  if(camera.available()) {
    camera.read(toReturn.buf, 4);
    uint8_t msg_len = toReturn.buf[2] - 1; // account for offsets
    Serial.print("MSG LEN: ");
    Serial.println(msg_len);

    camera.read(toReturn.buf, msg_len);
    for(int i = 1; i < msg_len; i++) {
      Serial.print((char)toReturn.buf[i]);
    }
    Serial.print("   CRC: ");
    Serial.println(toReturn.buf[msg_len]);


    if(msg_len != 255) {
      toReturn.status = 1;
      return toReturn;
    }
  }
  
  return toReturn;
}

