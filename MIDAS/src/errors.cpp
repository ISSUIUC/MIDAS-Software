#include "errors.h"
#include "lib/TCAL9539/TCAL9539.h"

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

#define TURN_ONE_RED_LED 
#define GPIO_EXPANDER_2 2

GpioAddress blue_led(GPIO_EXPANDER_2, 13);
GpioAddress green_led(GPIO_EXPANDER_2, 14);
GpioAddress orange_led(GPIO_EXPANDER_2, 15);
GpioAddress red_led(GPIO_EXPANDER_2, 16);

void update_error_LED(ErrorCode error) {
    switch (error) {
        case LowGCouldNotBeInitialized:
            gpioDigitalWrite(blue_led, HIGH);
            gpioDigitalWrite(green_led, LOW);
            gpioDigitalWrite(orange_led, LOW);
            gpioDigitalWrite(red_led, LOW);
            break;
        case SDBeginFailed:
            gpioDigitalWrite(blue_led, LOW);
            gpioDigitalWrite(green_led, HIGH);
            gpioDigitalWrite(orange_led, LOW);
            gpioDigitalWrite(red_led, LOW);
            break;
        case LowGRangeCouldNotBeSet:
            gpioDigitalWrite(blue_led, LOW);
            gpioDigitalWrite(green_led, LOW);
            gpioDigitalWrite(orange_led, HIGH);
            gpioDigitalWrite(red_led, LOW);
            break;
        case LowGODRLPFCouldNotBeSet:
            gpioDigitalWrite(blue_led, LOW);
            gpioDigitalWrite(green_led, LOW);
            gpioDigitalWrite(orange_led, LOW);
            gpioDigitalWrite(red_led, HIGH);
            break;
        case HighGCouldNotBeInitialized:
            gpioDigitalWrite(blue_led, HIGH);
            gpioDigitalWrite(green_led, HIGH);
            gpioDigitalWrite(orange_led, LOW);
            gpioDigitalWrite(red_led, LOW);
            break;   
        case HighGCoulNotUpdateDataRate:
            gpioDigitalWrite(blue_led, HIGH);
            gpioDigitalWrite(green_led, LOW);
            gpioDigitalWrite(orange_led, HIGH);
            gpioDigitalWrite(red_led, LOW);
            break;
        case MagnetometerCoultNotBeInitialized:
            gpioDigitalWrite(blue_led, HIGH);
            gpioDigitalWrite(green_led, LOW);
            gpioDigitalWrite(orange_led, LOW);
            gpioDigitalWrite(red_led, HIGH);
            break;
        case GyroCouldNotBeInitialized:
            gpioDigitalWrite(blue_led, LOW);
            gpioDigitalWrite(green_led, HIGH);
            gpioDigitalWrite(orange_led, HIGH);
            gpioDigitalWrite(red_led, LOW);
            break; 
        case GPSCouldNotBeInitialized:
            gpioDigitalWrite(blue_led, LOW);
            gpioDigitalWrite(green_led, HIGH);
            gpioDigitalWrite(orange_led, LOW);
            gpioDigitalWrite(red_led, HIGH);
            break; 
        case ContinuityCouldNotBeInitialized:
            gpioDigitalWrite(blue_led, LOW);
            gpioDigitalWrite(green_led, LOW);
            gpioDigitalWrite(orange_led, HIGH);
            gpioDigitalWrite(red_led, HIGH);
            break; 
        case CannotConnectBNO:
            gpioDigitalWrite(blue_led, HIGH);
            gpioDigitalWrite(green_led, HIGH);
            gpioDigitalWrite(orange_led, HIGH);
            gpioDigitalWrite(red_led, LOW);
            break; 
        case CannotInitBNO:
            gpioDigitalWrite(blue_led, HIGH);
            gpioDigitalWrite(green_led, HIGH);
            gpioDigitalWrite(orange_led, LOW);
            gpioDigitalWrite(red_led, HIGH);
            break; 
        default:
            gpioDigitalWrite(blue_led, HIGH);
            gpioDigitalWrite(green_led, HIGH);
            gpioDigitalWrite(orange_led, HIGH);
            gpioDigitalWrite(red_led, HIGH);
            break;
    }
}