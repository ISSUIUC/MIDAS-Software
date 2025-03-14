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

#define CAM_1 0
#define CAM_2 1

#define SIDE_CAMERA CAM_1
#define BULKHEAD_CAMERA CAM_2


enum class CameraCommand {
    CAMERA1_OFF = 0,
    CAMERA1_ON = 1,
    CAMERA2_OFF = 2,
    CAMERA2_ON = 3,
    VTX_OFF = 4,
    VTX_ON = 5,
    MUX_1 = 6,
    MUX_2 = 7
};

struct CameraB2B {

    void camera_on(int cam_index);
    void camera_off(int cam_index);
    void camera_toggle(int cam_index);

    void vtx_on();
    void vtx_off();
    void vtx_toggle();

    void vmux_set(int cam_select);
    void vmux_toggle();

    void update_cam_board_state();
    uint8_t cam_board_state;

    private:
    void transmit_command(CameraCommand command);
    bool cam_state_[2] = { false, false }; // false: off, true: on
    bool vtx_state_ = false;
    bool mux_select_ = false; // false: VMUX 1, true: VMUX 2
};

/**
 * @struct Interface for B2B communication protocol
 */
struct B2BInterface {
    ErrorCode init();

    CameraB2B camera;
};
