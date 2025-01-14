#include <Wire.h>
#include <SPI.h>

#include "buzzer_backend.h"
#include "led_backend.h"
#include "pyro_backend.h"
#include "sensors.h"
#include "TCAL9539.h"

#include "systems.h"
#include "hardware/pins.h"
#include "hardware/SDLog.h"
#include "telemetry_backend.h"

/**
 * Sets the config file and then starts all the threads using the config.
 */

#ifdef IS_SUSTAINER
// MultipleLogSink<EMMCSink> sinks;
MultipleLogSink<SDSink> sinks;
#else
MultipleLogSink<> sinks;
#endif

LowGSensor low_g_sensor;
LowGLSMSensor low_g_lsm_sensor;
HighGSensor high_g_sensor;
BarometerSensor barometer_sensor;
ContinuitySensor continuity_sensor;
VoltageSensor voltage_sensor;
OrientationSensor orientation_sensor;
MagnetometerSensor magnetometer_sensor;
GPSSensor gps_sensor;
TelemetryBackend telemetry_backend;
BuzzerBackend buzzer_backend;
LedBackend led_backend;
PyroBackend pyro_backend;

Sensors sensors {
    .low_g = low_g_sensor,
    .low_g_lsm = low_g_lsm_sensor,
    .high_g = high_g_sensor,
    .barometer = barometer_sensor,
    .continuity = continuity_sensor,
    .voltage = voltage_sensor,
    .orientation = orientation_sensor,
    .magnetometer = magnetometer_sensor,
    .gps = gps_sensor,
    .sink = sinks,
    .telemetry = telemetry_backend,
    .buzzer = buzzer_backend,
    .led = led_backend,
    .pyro = pyro_backend
};

RocketSystems rocket(sensors);

/**
 * @brief Sets up pinmodes for all sensors and starts threads
*/

void setup() {
    //begin serial port
    Serial.begin(9600);

//    while (!Serial);

    delay(200);

    //begin sensor SPI bus
    Serial.println("Starting SPI...");
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    //begin I2C bus
    Serial.println("Starting I2C...");
    Wire.begin(I2C_SDA, I2C_SCL);

    //set all chip selects high (deselected)
    pinMode(MS5611_CS, OUTPUT);
    pinMode(LSM6DS3_CS, OUTPUT);
    pinMode(KX134_CS, OUTPUT);
    pinMode(ADXL355_CS, OUTPUT);
    pinMode(LIS3MDL_CS, OUTPUT);
    pinMode(BNO086_CS, OUTPUT);
    pinMode(CAN_CS, OUTPUT);
    pinMode(RFM96_CS, OUTPUT);
    digitalWrite(MS5611_CS, HIGH);
    digitalWrite(LSM6DS3_CS, HIGH);
    digitalWrite(KX134_CS, HIGH);
    digitalWrite(ADXL355_CS, HIGH);
    digitalWrite(LIS3MDL_CS, HIGH);
    digitalWrite(BNO086_CS, HIGH);
    digitalWrite(CAN_CS, HIGH);
    digitalWrite(RFM96_CS, HIGH);

    //configure output leds
    gpioPinMode(LED_BLUE, OUTPUT);
    gpioPinMode(LED_GREEN, OUTPUT);
    gpioPinMode(LED_ORANGE, OUTPUT);
    gpioPinMode(LED_RED, OUTPUT);

    delay(200);

    //init and start threads
    rocket.begin();
}

void loop() {

}
