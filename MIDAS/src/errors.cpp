#include "errors.h"
#include "lib/TCAL9539/TCAL9539.h"

/**
 * If an error during initialization was detected, some combination of the blue, green, orange, and red LEDs will be on,
 * The combination of all 4 lights will tell you what the error is
 *
 * NONE => Nothing is wrong
 * BLUE => LowG Error
 * GREEN => Could not begin SD card
 * ORANGE => HighG error
 * RED => MAGNETOMETER error
 * BLUE, GREEN => GYRO error
 * BLUE, ORANGE => GPS error
 * BLUE, RED => CONTINUITY error
 * GREEN, ORANGE => BNO error
 * GREEN, RED => 
 * ORANGE, RED => 
 * BLUE, GREEN, ORANGE => 
 * BLUE, GREEN, RED => 
 * BLUE, ORANGE, RED => 
 * GREEN, ORANGE, RED =>
 * BLUE, GREEN, ORANGE, RED => Default, means something went very wrong
 */

#define TURN_ONE_RED_LED 
#define GPIO_EXPANDER_2 2

// Each LED pin, with pin humbers in octal
GpioAddress blue_led(GPIO_EXPANDER_2, 013);
GpioAddress green_led(GPIO_EXPANDER_2, 014);
GpioAddress orange_led(GPIO_EXPANDER_2, 015);
GpioAddress red_led(GPIO_EXPANDER_2, 016);

void update_error_LED(ErrorCode error) {
    switch (error) {
        case LowGCouldNotBeInitialized:
        case LowGRangeCouldNotBeSet:
        case LowGODRLPFCouldNotBeSet:
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
        case HighGCouldNotBeInitialized:
        case HighGCoulNotUpdateDataRate:
            gpioDigitalWrite(blue_led, LOW);
            gpioDigitalWrite(green_led, LOW);
            gpioDigitalWrite(orange_led, HIGH);
            gpioDigitalWrite(red_led, LOW);
            break;
        case MagnetometerCoultNotBeInitialized:
            gpioDigitalWrite(blue_led, LOW);
            gpioDigitalWrite(green_led, LOW);
            gpioDigitalWrite(orange_led, LOW);
            gpioDigitalWrite(red_led, HIGH);
            break;
        case GyroCouldNotBeInitialized:
            gpioDigitalWrite(blue_led, HIGH);
            gpioDigitalWrite(green_led, HIGH);
            gpioDigitalWrite(orange_led, LOW);
            gpioDigitalWrite(red_led, LOW);
            break;   
        case GPSCouldNotBeInitialized:
            gpioDigitalWrite(blue_led, HIGH);
            gpioDigitalWrite(green_led, LOW);
            gpioDigitalWrite(orange_led, HIGH);
            gpioDigitalWrite(red_led, LOW);
            break;
        case ContinuityCouldNotBeInitialized:
            gpioDigitalWrite(blue_led, HIGH);
            gpioDigitalWrite(green_led, LOW);
            gpioDigitalWrite(orange_led, LOW);
            gpioDigitalWrite(red_led, HIGH);
            break;
        case CannotConnectBNO:
        case CannotInitBNO:
            gpioDigitalWrite(blue_led, LOW);
            gpioDigitalWrite(green_led, HIGH);
            gpioDigitalWrite(orange_led, HIGH);
            gpioDigitalWrite(red_led, LOW);
            break;
        default:
            gpioDigitalWrite(blue_led, HIGH);
            gpioDigitalWrite(green_led, HIGH);
            gpioDigitalWrite(orange_led, HIGH);
            gpioDigitalWrite(red_led, HIGH);
            break;
    }
}
