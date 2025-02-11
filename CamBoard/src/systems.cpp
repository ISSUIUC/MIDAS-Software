#include "systems.h"
#include "hardware/voltage.h"

#include "hal.h"


/**
 * @brief These are all the functions that will run in each task
 * Each function has a `while (true)` loop within that should not be returned out of or yielded in any way
 *
 * The `DECLARE_THREAD` macro creates a function whose name is suffixed by _thread, and annotates it with [[noreturn]]
 */

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
            Serial.println(temp * 125 / 1000.0);
            Serial.print("Current ");
            Serial.println(current * 1.2 / 1000.0);
            Serial.print("Power ");
            Serial.println(power * 240 / 1000000.0);
        }
        arg->led.update();
        i += 1;

        THREAD_SLEEP(50);
    }
}

// This thread has a bit of extra logic since it needs to play a tune exactly once the sustainer ignites
DECLARE_THREAD(fsm, RocketSystems* arg) {
    FSM fsm{};
    // bool already_played_freebird = false;
    while (true) {
        FSMState current_state = arg->rocket_data.fsm_state;

        FSMState next_state = fsm.tick_fsm(current_state, arg);

        arg->rocket_data.fsm_state = next_state;

        // if (current_state == FSMState::STATE_ON && !already_played_freebird) {
        //     arg->buzzer.play_tune(free_bird, FREE_BIRD_LENGTH);
        //     already_played_freebird = true;
        // }

        THREAD_SLEEP(20);
    }
}

DECLARE_THREAD(buzzer, RocketSystems* arg) {
    while (true) {
        arg->buzzer.tick();

        THREAD_SLEEP(10);
    }
}

DECLARE_THREAD(can, RocketSystems* arg) {
    while (true) {
        CANFDMessage message;
        if (arg->can.recieve(message)) {
            arg->rocket_data.commands = (MIDASCommands({message}));
        }
        THREAD_SLEEP(5);
    }
}

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
    delay(3000);
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
    START_THREAD(buzzer, MAIN_CORE, config, 6);
    START_THREAD(can, MAIN_CORE, config, 15);

    config->buzzer.play_tune(free_bird, FREE_BIRD_LENGTH);

    while (true) {
        THREAD_SLEEP(1000);
    }
}

//void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char* pcTaskName){
//    Serial.println("OVERFLOW");
//    Serial.println((char*)pcTaskName);
//}