#include "b2b_interface.h"


ErrorCode B2BInterface::init() {
    // No special init
    return ErrorCode::NoError;
}

/** 
 * @brief Transmits the given CameraCommand over I2C / CAN (depending on which interface type is defined)
 */
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

/** 
 * @brief Transmits command to toggle on the video transmitter
 */
void CameraB2B::vtx_on() {
    transmit_command(CameraCommand::VTX_ON);
    vtx_state_ = true;
}

/** 
 * @brief Transmits command to toggle off the video transmitter
 */
void CameraB2B::vtx_off() {
    transmit_command(CameraCommand::VTX_OFF);
    vtx_state_ = false;
}

/** 
 * @brief Transmits command to toggle the video transmitter
 */
void CameraB2B::vtx_toggle() {
    if(vtx_state_) {
        vtx_off();
    } else {
        vtx_on();
    }
}

/** 
 * @brief Transmits command to set which camera is currently active on the video multiplexer
 */
void CameraB2B::vmux_set(int cam_select) {
    if(cam_select) {
        // If cam_select is 1, switch to MUX 2
        transmit_command(CameraCommand::MUX_2);
        mux_select_ = true;
    } else {
        // Otherwise switch to MUX 1
        transmit_command(CameraCommand::MUX_1);
        mux_select_ = false;
    }
}

/** 
 * @brief Transmits command to toggle which camera is currently active on the video multiplexer
 */
void CameraB2B::vmux_toggle() {
    vmux_set(!mux_select_);
}

/** 
 * @brief Transmits command to enable power to a camera
 */
void CameraB2B::camera_on(int cam_index) {
    switch (cam_index) {
        case 0:
            transmit_command(CameraCommand::CAMERA1_ON);
            cam_state_[0] = true;
            break;
        case 1:
            transmit_command(CameraCommand::CAMERA2_ON);
            cam_state_[1] = true;
            break;
        default:
            Serial.print("B2B camera on -- invalid index ");
            Serial.println(cam_index);  
            break;
    }
}

/** 
 * @brief Transmits command to disable power to a camera
 * 
 * If the camera was previously on, it will first stop recording, then power off.
 */
void CameraB2B::camera_off(int cam_index) {
    switch (cam_index) {
        case 0:
            transmit_command(CameraCommand::CAMERA1_OFF);
            cam_state_[0] = false;
            break;
        case 1:
            transmit_command(CameraCommand::CAMERA2_OFF);
            cam_state_[1] = false;
            break;
        default:
            Serial.print("B2B camera off -- invalid index ");
            Serial.println(cam_index);  
            break;
    }
}

/** 
 * @brief Transmits command to toggle power to a camera
 */
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
            Serial.print("B2B camera toggle -- invalid index ");
            Serial.println(cam_index);  
            break;
    }
}