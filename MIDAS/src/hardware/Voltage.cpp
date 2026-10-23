#include "sensors.h"
#include <Arduino.h>
#include <ads7138-q1.h>
#include <Wire.h>

#define ADC_I2C_ADDR 0x14

/// Global instance of the ADS7138-Q1 analog-to-digital converter.
ADS7138 ADC;

/**
 * @brief Initializes the ADS7138 voltage monitoring ADC.
 *
 * Configures communication with the ADS7138-Q1 over the shared I2C bus.
 * This ADC is responsible for measuring the avionics battery voltage,
 * pyro battery voltage, and the continuity sense lines for each pyro
 * channel.
 *
 * @return ErrorCode::NoError if the ADC was initialized successfully.
 * @return ErrorCode::ADCFailedToInit if communication with the ADC
 *         could not be established.
 */
ErrorCode VoltageSensor::init() {

    // Attempt to initialize the ADC on the configured I2C address.
    if (!ADC.init(&Wire, ADC_I2C_ADDR)) {
        return ErrorCode::ADCFailedToInit;
    }

    return ErrorCode::NoError;
}

/**
 * @brief Reads all voltage monitoring channels from the ADS7138.
 *
 * Samples every analog channel used by the flight computer, including
 * the avionics battery, pyro battery, and continuity sensing circuitry
 * for each pyro output. The returned values are packaged into a
 * Voltage data structure for downstream use by the flight software.
 *
 * @return Voltage structure containing the latest ADC measurements.
 */
Voltage VoltageSensor::read() {
    Voltage voltage;

    // Update the ADC's internal conversion state.
    ADC.tick();

    // Read the avionics battery voltage.
    voltage.v_Bat = ADC.read(VBAT_SENSE);

    // Read the dedicated pyro battery voltage.
    voltage.v_Pyro = ADC.read(PYRO_SENSE);

    // Read continuity sense voltages for each pyro channel.
    voltage.continuity[0] = ADC.read(SENSE_A);
    voltage.continuity[1] = ADC.read(SENSE_B);
    voltage.continuity[2] = ADC.read(SENSE_C);
    voltage.continuity[3] = ADC.read(SENSE_D);

    return voltage;
}