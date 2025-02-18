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

void CameraB2B::vtx_on() {
    transmit_command(CameraCommand::VTX_ON);
    vtx_state_ = true;
}

void CameraB2B::vtx_off() {
    transmit_command(CameraCommand::VTX_OFF);
    vtx_state_ = false;
}

void CameraB2B::vtx_toggle() {
    if(vtx_state_) {
        vtx_off();
    } else {
        vtx_on();
    }
}

void CameraB2B::camera_on(int cam_index) {
    switch (cam_index) {
        case 0:
            transmit_command(CameraCommand::CAMERA0_ON);
            cam_state_[0] = true;
            break;
        case 1:
            transmit_command(CameraCommand::CAMERA1_ON);
            cam_state_[1] = true;
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
            cam_state_[0] = false;
            break;
        case 1:
            transmit_command(CameraCommand::CAMERA1_OFF);
            cam_state_[1] = false;
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
                camera_off(0);
            } else {
                camera_on(0);
            }
            break;
        case 1:
            if (cam_state_[1]) {
                camera_off(1);
            } else {
                camera_on(1);
            }
            break;
        default:
            Serial.print("B2B camera on -- invalid index ");
            Serial.println(cam_index);  
            break;
    }
}