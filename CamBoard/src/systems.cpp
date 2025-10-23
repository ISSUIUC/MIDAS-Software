#include "systems.h"
#include "EEPROM.h"
#include "hardware/voltage.h"

#include "Queue.h"
#include "hal.h"

// Amount of time (in ms) without communication that causes us to go into the i2c recovery state.
#define I2C_RECOVERY_STATE_THRESHOLD 3000

// Amount of time that, if spent in the recovery state, we instead fallback to "ALL CAMS ON"
// Defaults to 3 min
#define I2C_RECOVERY_FALLBACK_TIME 180000

cam_state_t GLOBAL_CAM_STATE;
cam_state_t DESIRED_CAM_STATE;
uint32_t LAST_I2C_COMM;

Queue<uint8_t> commandqueue;

/**
 * @brief These are all the functions that will run in each task
 * Each function has a `while (true)` loop within that should not be returned out of or yielded in any way
 *
 * The `DECLARE_THREAD` macro creates a function whose name is suffixed by _thread, and annotates it with [[noreturn]]
 */

// Handles incoming command queue
DECLARE_THREAD(cmdq, RocketSystems* arg) {
    uint8_t cur_cmd;
    while (true) {
        if (commandqueue.receive(&cur_cmd)) {
            switch(cur_cmd) {
            case 0: {
                Serial.println("proc cmd: CAM1 OFF");
                camera_on_off(Serial1); // Stop recording
                DESIRED_CAM_STATE.cam1_on = false; // Attempt to turn off the camera when recording stopped.
                delay(5000);
                digitalWrite(CAM1_ON_OFF, LOW); // force power off after 5s
                break;}
            case 1: {
                Serial.println("proc cmd: CAM1 ON");
                DESIRED_CAM_STATE.cam1_on = true;
                digitalWrite(CAM1_ON_OFF, HIGH);
                break;}
            case 2:{
                Serial.println("proc cmd: CAM2 OFF");
                camera_on_off(Serial2); // Stop recording
                DESIRED_CAM_STATE.cam2_on = false; // Attempt to turn off the camera when recording stopped.
                delay(5000);
                digitalWrite(CAM2_ON_OFF, LOW); // force power off after 5s
                break;}
            case 3: {
                Serial.println("proc cmd: CAM2 ON");
                DESIRED_CAM_STATE.cam2_on = true;
                digitalWrite(CAM2_ON_OFF, HIGH);
                break;}
            case 4:
                digitalWrite(VTX_ON_OFF, LOW);
                Serial.println("proc cmd: VTX ON/OFF --> LOW");
                GLOBAL_CAM_STATE.vtx_on = false;
                DESIRED_CAM_STATE.vtx_on = false;
                break;
            case 5:
                digitalWrite(VTX_ON_OFF, HIGH);
                Serial.println("proc cmd: VTX ON/OFF --> HIGH");
                GLOBAL_CAM_STATE.vtx_on = true;
                DESIRED_CAM_STATE.vtx_on = true;
                break;
            case 6:
                digitalWrite(VIDEO_SELECT, LOW);
                Serial.println("proc cmd: VIDEO SELECT --> LOW");
                GLOBAL_CAM_STATE.vmux_state = false;
                break;
            case 7:
                digitalWrite(VIDEO_SELECT, HIGH);
                Serial.println("proc cmd: VIDEO_SELECT --> HIGH");
                GLOBAL_CAM_STATE.vmux_state = true;
                break;
            default:
                break;
            }
        }
        THREAD_SLEEP(BROWNOUT_PROTECTION_DELAY);
    }
}

// Ever device which communicates over i2c is on this thread to avoid interference
DECLARE_THREAD(i2c, RocketSystems* arg) {
    int i = 0;

    while (true) {
        if (i % 10 == 0) {
            int power = read_reg(0x8, 3);
            int current = read_reg(0x7, 2);
            int temp = read_reg(0x6, 2);
            int voltage = read_reg(0x5, 2);
            //arg->rocket_data.voltage_sense.update({power, current, temp, voltage});
            Serial.print("Voltage ");
            Serial.println(voltage * 3.125 / 1000.0);
            Serial.print("Temp ");
            Serial.println((temp>>4) * 125 / 1000.0);
            Serial.print("Current ");
            Serial.println(current * 1.2 / 1000.0);
            Serial.print("Power ");
            Serial.println(power * 240 / 1000000.0);

            //read_mem_cap_data(Serial1);

          // int change = 0;

          // while(change == 0) {
          //   Serial.println("Trying to turn on camera");
          //   digitalWrite(CAM1_ON_OFF, HIGH);
            
          //   struct read_mem_cap_data_return toReturn1;
          //   toReturn1 = read_mem_cap_data(Serial1);
          //   while(toReturn1.status == 0) {
          //     toReturn1 = read_mem_cap_data(Serial1);
          //   }

          //   struct read_mem_cap_data_return toReturn2;
          //   toReturn2 = read_mem_cap_data(Serial1);
          //   while(toReturn2.status == 0) {
          //     toReturn2 = read_mem_cap_data(Serial1);
          //   }

          //   for(int i = 0; i < 12; i++) {
          //     if(toReturn1.buf[i] != toReturn2.buf[2]) {
          //       change = 1;
          //       break;
          //     }
          //   }

          //   if(change == 0) {
          //     camera_on_off(Serial1);
          //   }
          //   delay(5000);
          // }
        }
        arg->led.update();
        i += 1;

        THREAD_SLEEP(100);
    }
}

DECLARE_THREAD(comms_check, RocketSystems* arg) {
    bool is_in_recovery_state = false;
    bool is_in_fallback_state = false;
    uint32_t time_entered_fallback_state = millis();
    uint32_t recovery_debounce = millis();
    uint32_t fail_detect_debounce = millis();

    while(true) {
        uint32_t cur_time = millis();
        // The other half of I2C stuff -- checking communication.
        if(is_in_recovery_state) {
            if(cur_time > recovery_debounce) {
                Serial.println("Attempting recovery...");
                recovery_debounce = cur_time + 5000; // Only attempt recovery every 3 seconds.
                // Attempt recovery.
                Wire1.begin((uint8_t)CAMBOARD_I2C_ADDR);
                Serial.println("Sleeping for recovery test...");
                THREAD_SLEEP(500);

                if(millis() - LAST_I2C_COMM <= I2C_RECOVERY_STATE_THRESHOLD) {
                    Serial.println("Successfully recovered!");
                    digitalWrite(LED_ORANGE, LOW);
                    fail_detect_debounce = millis() + 3000; // After successful recovery, we only detect fault after 3 more sec
                    is_in_recovery_state = false;
                    is_in_fallback_state = false;
                    digitalWrite(LED_RED, LOW);
                } else {
                    Wire1.end();
                    pinMode(I2C_SCL, INPUT);
                    pinMode(I2C_SDA, INPUT);
                    Serial.println("Failed to recover.");
                }
            }

            if(cur_time - time_entered_fallback_state > I2C_RECOVERY_FALLBACK_TIME && !is_in_fallback_state) {
                is_in_fallback_state = true;
                digitalWrite(LED_RED, HIGH);
                arg->buzzer.play_tune(beep_beep, BEEP_LENGTH);
                delay(50);
                digitalWrite(CAM1_ON_OFF, HIGH);
                delay(50);
                digitalWrite(CAM2_ON_OFF, HIGH);
                delay(50);
                digitalWrite(VTX_ON_OFF, HIGH);
                digitalWrite(VIDEO_SELECT, LOW);

                DESIRED_CAM_STATE.cam1_on = true;
                DESIRED_CAM_STATE.cam2_on = true;
                DESIRED_CAM_STATE.cam1_rec = true;
                DESIRED_CAM_STATE.cam2_rec = true;
                DESIRED_CAM_STATE.vtx_on = true;
                DESIRED_CAM_STATE.vmux_state = false;
            }
        }

        if(cur_time - LAST_I2C_COMM > I2C_RECOVERY_STATE_THRESHOLD && !is_in_recovery_state && cur_time > fail_detect_debounce) {
            is_in_recovery_state = true;
            time_entered_fallback_state = cur_time;
            digitalWrite(LED_ORANGE, HIGH);
            Serial.println("I2C Comms not seen!");
            Wire1.end();

            pinMode(I2C_SCL, INPUT);
            pinMode(I2C_SDA, INPUT);
            Serial.println("Entered recovery mode");
        }
        // Serial.println("bruh5");
        THREAD_SLEEP(10);
        // Serial.println("bruh6");
    }
}

DECLARE_THREAD(flash, RocketSystems* arg) {

    while (true) {
        // Flash memory will store the desired state of the cameras as a single byte.
        // We will just store DESIRED_STATE and read from it on power on.

        // Convert DESIRED_STATE to a single byte
        uint8_t desired_state = 0;
        desired_state |= (0b00000001 & DESIRED_CAM_STATE.cam1_on);
        desired_state |= (0b00000010 & (DESIRED_CAM_STATE.cam2_on << 1));
        desired_state |= (0b00000100 & (DESIRED_CAM_STATE.vtx_on << 2));
        desired_state |= (0b00001000 & (DESIRED_CAM_STATE.vmux_state << 3));

        // Cameras should always be recording if they're on, but we can add that data here anyway
        desired_state |= (0b00010000 & (DESIRED_CAM_STATE.cam1_rec << 4));
        desired_state |= (0b00100000 & (DESIRED_CAM_STATE.cam2_rec << 5));

        EEPROM.write(0, desired_state);
        EEPROM.commit();
        THREAD_SLEEP(500);
    }
}



// This thread has a bit of extra logic since it needs to play a tune exactly once the sustainer ignites
DECLARE_THREAD(fsm, RocketSystems* arg) {
    // Data and telemetry thread
    size_t current_mem_cam1 = 0;
    size_t cam1_consecutive_invalid = 0;
    size_t current_mem_cam2 = 0;
    size_t cam2_consecutive_invalid = 0;

    while (true) {
        
        // Read all camera state
        // ignore all errors, try again next

        // CAMERA 1 STATE
        struct read_mem_cap_data_return toReturn1;
        Serial.println("CAM1 reading...");
        toReturn1 = read_mem_cap_data(Serial1);
        if (toReturn1.status == 1) {
            Serial.print("read: ");
            Serial.println(toReturn1.status);
            GLOBAL_CAM_STATE.cam1_on = true;
            cam1_consecutive_invalid = 0;

            if(current_mem_cam1 == 0) {
                Serial.println("first capture!");
                current_mem_cam1 = toReturn1.mem_size;
            } else {
                Serial.println("valid memory capture!");
                if (current_mem_cam1 != toReturn1.mem_size) {
                    Serial.println("is recording!");
                    GLOBAL_CAM_STATE.cam1_rec = true;
                } else {
                    GLOBAL_CAM_STATE.cam1_rec = false;
                }
                current_mem_cam1 = toReturn1.mem_size;
            }
        } else {
            Serial.println("bad read");
            cam1_consecutive_invalid++;
            if(cam1_consecutive_invalid >= 3) {
                GLOBAL_CAM_STATE.cam1_on = false;
                GLOBAL_CAM_STATE.cam1_rec = false;
            }
        }
        
        // CAMERA 2 STATE
        struct read_mem_cap_data_return toReturn2;
        Serial.println("CAM2 reading...");
        toReturn2 = read_mem_cap_data(Serial2);
        
        if (toReturn2.status == 1) {
            Serial.print("Cam2 read: ");
            Serial.println(toReturn2.status);
            GLOBAL_CAM_STATE.cam2_on = true;
            cam2_consecutive_invalid = 0;

            if(current_mem_cam2 == 0) {
                Serial.println("Cam2 first capture!");
                current_mem_cam2 = toReturn2.mem_size;
            } else {
                Serial.println("cam2 valid memory capture!");
                if (current_mem_cam2 != toReturn2.mem_size) {
                    GLOBAL_CAM_STATE.cam2_rec = true;
                } else {
                    GLOBAL_CAM_STATE.cam2_rec = false;
                }
                current_mem_cam2 = toReturn2.mem_size;
            }
        } else {
            Serial.println("cam2 invalid memory capture!");
            cam2_consecutive_invalid++;
            if(cam2_consecutive_invalid >= 3) {
                GLOBAL_CAM_STATE.cam2_on = false;
                GLOBAL_CAM_STATE.cam2_rec = false;
            }
        }


    

        // if the read is good for cam1
        // set cam1_rec to whether its recording or not

        if(DESIRED_CAM_STATE.cam1_on == false && GLOBAL_CAM_STATE.cam1_on == true) {
            // If we want to trun cam1 off, only do so when it has stopped recording.
            if(!GLOBAL_CAM_STATE.cam1_rec) {
              digitalWrite(CAM1_ON_OFF, LOW);
              Serial.println("camera 1 attempting to turn off");
            }
        }


        // if the read is good for cam2
        // set cam2_rec to whether its recording or not

        if(DESIRED_CAM_STATE.cam2_on == false && GLOBAL_CAM_STATE.cam2_on == true) {
            // If we want to trun cam2 off, only do so when it has stopped recording.
            if(!GLOBAL_CAM_STATE.cam2_rec) {
              digitalWrite(CAM2_ON_OFF, LOW);
              Serial.println("camera 2 attempting to turn off");
            }
        }


        THREAD_SLEEP(100);
    }
}

DECLARE_THREAD(buzzer, RocketSystems* arg) {
    while (true) {
        arg->buzzer.tick();

        THREAD_SLEEP(10);
    }
}

// DECLARE_THREAD(can, RocketSystems* arg) {
//     while (true) {
//         CANFDMessage message;
//         if (arg->can.recieve(message)) {
//             arg->rocket_data.commands = (MIDASCommands({message}));
//         }
//         THREAD_SLEEP(5);
//     }
// }

// DECLARE_THREAD(camera, RocketSystems* arg) {
//     while (true) {
//         THREAD_SLEEP(10);
//     }
// }


#define INIT_SYSTEM(s) do { ErrorCode code = (s).init(); if (code != NoError) { return code; } } while (0)

/**
 * @brief Initializes all systems in order, returning early if a system's initialization process errors out.
 *        Turns on the Orange LED while initialization is running.
 */
ErrorCode init_systems(RocketSystems& systems) {
    digitalWrite(LED_ORANGE, HIGH);

    // INIT_SYSTEM(systems.sensors.voltage);
    INIT_SYSTEM(systems.can);
    INIT_SYSTEM(systems.led);
    INIT_SYSTEM(systems.buzzer);
    INIT_SYSTEM(systems.cameras);


    // Just a short delay
    delay(500);
    Serial.println("Finish setup");
    digitalWrite(LED_ORANGE, LOW);

    

    return NoError;
}
#undef INIT_SYSTEM


/**
 * @brief Initializes the systems, and then creates and starts the thread for each system.
 *        If initialization fails, then this enters an infinite loop.
 */
[[noreturn]] void begin_systems(RocketSystems* config) {
    Serial.println("Starting Systems...");

    ErrorCode init_error_code = init_systems(*config);
    if (init_error_code != NoError) {
        // todo some message probably
        while (true) {
            Serial.print("Had Error: ");
            Serial.print((int) init_error_code);
            Serial.print("\n");
            Serial.flush();
            update_error_LED(init_error_code);
        }
    }

    START_THREAD(i2c, MAIN_CORE, config, 9);
    START_THREAD(fsm, MAIN_CORE, config, 8);
    START_THREAD(cmdq, MAIN_CORE, config, 7);
    START_THREAD(buzzer, MAIN_CORE, config, 6);
    START_THREAD(flash, MAIN_CORE, config, 10);
    START_THREAD(comms_check, MAIN_CORE, config, 10);
    
    //START_THREAD(can, MAIN_CORE, config, 15);

    config->buzzer.play_tune(free_bird, FREE_BIRD_LENGTH);

    while (true) {
        THREAD_SLEEP(1000);
    }
}

//void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char* pcTaskName){
//    Serial.println("OVERFLOW");
//    Serial.println((char*)pcTaskName);
//}