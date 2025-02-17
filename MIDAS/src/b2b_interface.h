#include <stdint.h>
#include <stddef.h>
#include <Wire.h>
#include "errors.h"
#include "hal.h"

// Which b2b communication we should use
#define B2B_I2C
// #define B2B_CAN

#if defined(B2B_I2C) && defined(B2B_CAN)
#error "B2B can only use one option of B2B_I2C, or B2B_CAN"
#elif !defined(B2B_I2C) && !defined(B2B_CAN)
#error "At least one B2B_I2C or B2B_CAN must be defined"
#endif


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

struct CameraB2B {

    void camera_on(int cam_index);
    void camera_off(int cam_index);
    void camera_toggle(int cam_index);

    private:
    void transmit_command(CameraCommand command);
    bool cam_state_[2] = { false, false }; // false: off, true: on
};

/**
 * @struct Interface for B2B communication protocol
 */
struct B2BInterface {
    ErrorCode init();

    CameraB2B camera;
};
