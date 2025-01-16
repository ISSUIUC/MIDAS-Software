#include "errors.h"

/**
 * If an error during initialization was detected, some combination of the blue, green, orange, and red LEDs will be on,
 * The combination of all 4 lights will tell you what the error is
 *
 * NONE => Nothing is wrong
 * BLUE => LOW G could not init
 * GREEN => Could not begin SD card
 * ORANGE => Could not set LOW G range
 * RED => LOW G Output Data Rate Low Pass Filter could not be set
 * BLUE, GREEN => HIGH G could not init
 * BLUE, ORANGE => HIGH G could not update Data Rate
 * BLUE, RED => MAGNETOMETER could not be init
 * GREEN, ORANGE => GYRO could not init
 * GREEN, RED => GPS could not init
 * ORANGE, RED => CONTINUITY could not init
 * BLUE, GREEN, ORANGE => BNO could not connect
 * BLUE, GREEN, RED => BNO could not init
 * BLUE, ORANGE, RED => 
 * GREEN, ORANGE, RED =>
 * BLUE, GREEN, ORANGE, RED => Default, somehow the function was called with a different error, should never happen
 */

#include <hardware_interface.h>

#include "led.h"

/**
 * @brief writes LEDS to indicate errors as described above
 */
void update_error_LED(ILedBackend& led, ErrorCode error) {
    switch (error) {
        case LowGCouldNotBeInitialized:
            led.set(LED::BLUE, true);
            led.set(LED::GREEN, false);
            led.set(LED::ORANGE, false);
            led.set(LED::RED, false);
            break;
        case SDBeginFailed:
            led.set(LED::BLUE, false);
            led.set(LED::GREEN, true);
            led.set(LED::ORANGE, false);
            led.set(LED::RED, false);
            break;
        case LowGRangeCouldNotBeSet:
            led.set(LED::BLUE, false);
            led.set(LED::GREEN, false);
            led.set(LED::ORANGE, true);
            led.set(LED::RED, false);
            break;
        case LowGODRLPFCouldNotBeSet:
            led.set(LED::BLUE, false);
            led.set(LED::GREEN, false);
            led.set(LED::ORANGE, false);
            led.set(LED::RED, true);
            break;
        case HighGCouldNotBeInitialized:
            led.set(LED::BLUE, true);
            led.set(LED::GREEN, true);
            led.set(LED::ORANGE, false);
            led.set(LED::RED, false);
            break;   
        case HighGCouldNotUpdateDataRate:
            led.set(LED::BLUE, true);
            led.set(LED::GREEN, false);
            led.set(LED::ORANGE, true);
            led.set(LED::RED, false);
            break;
        case MagnetometerCouldNotBeInitialized:
            led.set(LED::BLUE, true);
            led.set(LED::GREEN, false);
            led.set(LED::ORANGE, false);
            led.set(LED::RED, true);
            break;
        case GyroCouldNotBeInitialized:
            led.set(LED::BLUE, false);
            led.set(LED::GREEN, true);
            led.set(LED::ORANGE, true);
            led.set(LED::RED, false);
            break; 
        case GPSCouldNotBeInitialized:
            led.set(LED::BLUE, false);
            led.set(LED::GREEN, true);
            led.set(LED::ORANGE, false);
            led.set(LED::RED, true);
            break; 
        case ContinuityCouldNotBeInitialized:
            led.set(LED::BLUE, false);
            led.set(LED::GREEN, false);
            led.set(LED::ORANGE, true);
            led.set(LED::RED, true);
            break; 
        case CannotConnectBNO:
            led.set(LED::BLUE, true);
            led.set(LED::GREEN, true);
            led.set(LED::ORANGE, true);
            led.set(LED::RED, false);
            break; 
        case CannotInitBNO:
            led.set(LED::BLUE, true);
            led.set(LED::GREEN, true);
            led.set(LED::ORANGE, false);
            led.set(LED::RED, true);
            break; 
        default:
            led.set(LED::BLUE, true);
            led.set(LED::GREEN, true);
            led.set(LED::ORANGE, true);
            led.set(LED::RED, true);
            break;
    }
}
