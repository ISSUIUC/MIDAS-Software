#include "errors.h"
#include "Arduino.h"
//#include "TCAL9539.h"

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

#ifndef SILSIM
//#include <TCAL9539.h>
#include "hardware/pins.h"

/**
 * @brief writes LEDS to indicate errors as described above
 * 
 * @param error Error Code to indicate
*/
void update_error_LED(ErrorCode error) {
    switch (error) {
        // case LowGCouldNotBeInitialized:
        //     gpioDigitalWrite(LED_BLUE, HIGH);
        //     gpioDigitalWrite(LED_GREEN, LOW);
        //     gpioDigitalWrite(LED_ORANGE, LOW);
        //     gpioDigitalWrite(LED_RED, LOW);
        //     break;
        // case SDBeginFailed:
        //     gpioDigitalWrite(LED_BLUE, LOW);
        //     gpioDigitalWrite(LED_GREEN, HIGH);
        //     gpioDigitalWrite(LED_ORANGE, LOW);
        //     gpioDigitalWrite(LED_RED, LOW);
        //     break;
        // case LowGRangeCouldNotBeSet:
        //     gpioDigitalWrite(LED_BLUE, LOW);
        //     gpioDigitalWrite(LED_GREEN, LOW);
        //     gpioDigitalWrite(LED_ORANGE, HIGH);
        //     gpioDigitalWrite(LED_RED, LOW);
        //     break;
        // case LowGODRLPFCouldNotBeSet:
        //     gpioDigitalWrite(LED_BLUE, LOW);
        //     gpioDigitalWrite(LED_GREEN, LOW);
        //     gpioDigitalWrite(LED_ORANGE, LOW);
        //     gpioDigitalWrite(LED_RED, HIGH);
        //     break;
        // case HighGCouldNotBeInitialized:
        //     gpioDigitalWrite(LED_BLUE, HIGH);
        //     gpioDigitalWrite(LED_GREEN, HIGH);
        //     gpioDigitalWrite(LED_ORANGE, LOW);
        //     gpioDigitalWrite(LED_RED, LOW);
        //     break;   
        // case HighGCouldNotUpdateDataRate:
        //     gpioDigitalWrite(LED_BLUE, HIGH);
        //     gpioDigitalWrite(LED_GREEN, LOW);
        //     gpioDigitalWrite(LED_ORANGE, HIGH);
        //     gpioDigitalWrite(LED_RED, LOW);
        //     break;
        // case MagnetometerCouldNotBeInitialized:
        //     gpioDigitalWrite(LED_BLUE, HIGH);
        //     gpioDigitalWrite(LED_GREEN, LOW);
        //     gpioDigitalWrite(LED_ORANGE, LOW);
        //     gpioDigitalWrite(LED_RED, HIGH);
        //     break;
        // case GyroCouldNotBeInitialized:
        //     gpioDigitalWrite(LED_BLUE, LOW);
        //     gpioDigitalWrite(LED_GREEN, HIGH);
        //     gpioDigitalWrite(LED_ORANGE, HIGH);
        //     gpioDigitalWrite(LED_RED, LOW);
        //     break; 
        // case GPSCouldNotBeInitialized:
        //     gpioDigitalWrite(LED_BLUE, LOW);
        //     gpioDigitalWrite(LED_GREEN, HIGH);
        //     gpioDigitalWrite(LED_ORANGE, LOW);
        //     gpioDigitalWrite(LED_RED, HIGH);
        //     break; 
        // case ContinuityCouldNotBeInitialized:
        //     gpioDigitalWrite(LED_BLUE, LOW);
        //     gpioDigitalWrite(LED_GREEN, LOW);
        //     gpioDigitalWrite(LED_ORANGE, HIGH);
        //     gpioDigitalWrite(LED_RED, HIGH);
        //     break; 
        // case CannotConnectBNO:
        //     gpioDigitalWrite(LED_BLUE, HIGH);
        //     gpioDigitalWrite(LED_GREEN, HIGH);
        //     gpioDigitalWrite(LED_ORANGE, HIGH);
        //     gpioDigitalWrite(LED_RED, LOW);
        //     break; 
        // case CannotInitBNO:
        //     gpioDigitalWrite(LED_BLUE, HIGH);
        //     gpioDigitalWrite(LED_GREEN, HIGH);
        //     gpioDigitalWrite(LED_ORANGE, LOW);
        //     gpioDigitalWrite(LED_RED, HIGH);
        //     break; 
        default:
            digitalWrite(LED_BLUE, HIGH);
            digitalWrite(LED_GREEN, HIGH);
            digitalWrite(LED_ORANGE, HIGH);
            digitalWrite(LED_RED, HIGH);
            break;
    }
}
#else

void update_error_LED(ErrorCode error) {

};

#endif