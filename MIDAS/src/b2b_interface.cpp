#include "b2b_interface.h"


ErrorCode B2BInterface::init() {
    // No special init
    return ErrorCode::NoError;
}



void CameraB2B::transmit_command(CameraCommand command) {
    #ifdef B2B_I2C

    Wire.beginTransmission(0x69); // 0x69 --> Camera board i2c address
    Wire.write((uint8_t) command);
    if (Wire.endTransmission()) {
        Serial.println("Camera B2B i2c write error");
    }

    #endif

    #ifdef B2B_CAN
        // todo :D
    #endif
}

void CameraB2B::camera_on(int cam_index) {
    switch (cam_index) {
        case 0:
            transmit_command(CameraCommand::CAMERA0_ON);
            break;
        case 1:
            transmit_command(CameraCommand::CAMERA1_ON);
            break;
        default:
            Serial.print("B2B camera on -- invalid index ");
            Serial.println(cam_index);  
            break;
    }
}

void CameraB2B::camera_off(int cam_index) {
    switch (cam_index) {
        case 0:
            transmit_command(CameraCommand::CAMERA0_OFF);
            break;
        case 1:
            transmit_command(CameraCommand::CAMERA1_OFF);
            break;
        default:
            Serial.print("B2B camera on -- invalid index ");
            Serial.println(cam_index);  
            break;
    }
}

void CameraB2B::camera_toggle(int cam_index) {
    switch (cam_index) {
        case 0:
            if (cam_state_[0]) {
                transmit_command(CameraCommand::CAMERA0_OFF);
            } else {
                transmit_command(CameraCommand::CAMERA0_ON);
            }
            break;
        case 1:
            if (cam_state_[1]) {
                transmit_command(CameraCommand::CAMERA1_OFF);
            } else {
                transmit_command(CameraCommand::CAMERA1_ON);
            }
            break;
        default:
            Serial.print("B2B camera on -- invalid index ");
            Serial.println(cam_index);  
            break;
    }
}