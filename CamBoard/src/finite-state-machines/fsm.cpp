#include <cmath>

#include "fsm.h"
#include "hardware/camera.h"


/**
 * @brief Camboard FSM tick function, which will advance the current state if necessary
 * 
 * @param state current FSM state
 * @param arg Current status of CamBoard
 * 
 * @return New FSM State
*/
FSMState FSM::tick_fsm(FSMState& state, RocketSystems* arg) {

    switch (state) {
        case FSMState::STATE_IDLE:
            if (arg->rocket_data.commands.getRecent().command.data[0] == (uint8_t) 0) {
                state = FSMState::STATE_ON;
                camera_on_off(arg->cameras.cam1);
                camera_on_off(arg->cameras.cam2);
                start_recording(arg->cameras.cam1);
                start_recording(arg->cameras.cam2);
            }
            break;

        case FSMState::STATE_ON:
            if (arg->rocket_data.commands.getRecent().command.data[0] == (uint8_t) 1) {
                state = FSMState::STATE_RECORDING_ASCENT;
            }
            break;


        case FSMState::STATE_RECORDING_ASCENT:
            if (arg->rocket_data.commands.getRecent().command.data[0] == (uint8_t) 2) {
                state = FSMState::STATE_RECORDING_DESCENT;
                digitalWrite(VIDEO_SELECT, HIGH);
            }
            break;

        case FSMState::STATE_RECORDING_DESCENT:
            if (arg->rocket_data.commands.getRecent().command.data[0] == (uint8_t) 3) {
                state = FSMState::STATE_IDLE;
                stop_recording(arg->cameras.cam1);
                stop_recording(arg->cameras.cam2);
                camera_on_off(arg->cameras.cam1);
                camera_on_off(arg->cameras.cam2);
                
            }
            break;
    }
    return state;
}