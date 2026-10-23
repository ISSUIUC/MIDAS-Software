#pragma once

#include "flight-systems/sensor_data.h"
#include "finite-state-machines/fsm_config.h"

/**
 * @brief Defines the persistent EEPROM layout for the MIDAS flight computer.
 *
 * This structure represents all data stored in non-volatile memory. It is used
 * to generate the EEPROM checksum, allowing firmware to verify that stored data
 * is valid and compatible with the current EEPROM schema during startup.
 *
 * Any modification to this structure changes the EEPROM layout and should be
 * accompanied by an updated checksum to prevent invalid data from being loaded.
 */
struct MIDASEEPROM {
    /**
     * @brief Checksum used to validate the EEPROM contents.
     */
    uint32_t checksum;

    /**
     * @brief Number of the most recently created flight log.
     *
     * Used to continue log numbering across power cycles and prevent
     * overwriting previous flight data.
     */
    uint16_t sd_file_num_last = 0;

    /**
     * @brief Unique flight computer serial number.
     */
    uint8_t serial = 0;

    /**
     * @brief Default LoRa telemetry frequency in MHz.
     */
    float frequency = 421.15;

    /**
     * @brief High-G accelerometer calibration bias.
     *
     * Applied to compensate for sensor offset after calibration.
     */
    Acceleration lsm6dsv320x_hg_xl_bias = {0.0f, 0.0f, 0.0f};

    /**
     * @brief Magnetometer soft-iron calibration scale factors.
     *
     * Used to compensate for magnetic distortion caused by nearby
     * ferromagnetic materials.
     */
    Magnetometer mmc5983ma_softiron_bias = {1.0f, 1.0f, 1.0f};

    /**
     * @brief Magnetometer hard-iron calibration offsets.
     *
     * Used to remove constant magnetic field offsets introduced by
     * permanently magnetized components.
     */
    Magnetometer mmc5983ma_hardiron_bias = {0.0f, 0.0f, 0.0f};

    /**
     * @brief Stored finite state machine configuration.
     *
     * Contains configurable flight state transitions, thresholds,
     * timers, and pyro firing rules.
     */
    FSMConfiguration fsm_config;
};