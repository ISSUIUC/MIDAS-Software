#include <Wire.h>
#include <SPI.h>
#include "TCAL9538.h"

#include "systems.h"
#include "hardware/pins.h"
#include "hardware/Emmc.h"
#include "hardware/SDLog.h"
#include "sensor_data.h"
#include "pins.h"

/**
 * Sets the config file and then starts all the threads using the config.
 */

// #ifdef IS_SUSTAINER
// MultipleLogSink<EMMCSink> sinks;
MultipleLogSink<SDSink> sinks;
// #else
// MultipleLogSink<> sinks;
// #endif
RocketSystems systems{.log_sink = sinks};
/**
 * @brief Sets up pinmodes for all sensors and starts threads
 */

void setup()
{
    // begin serial port
    Serial.begin(9600);

    delay(200);

    // Immediate startup tone
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);

    #ifdef IS_SUSTAINER
    ledcWriteTone(BUZZER_CHANNEL, 3200);
    delay(250);
    ledcWriteTone(BUZZER_CHANNEL, 0);
    delay(100);
    ledcWriteTone(BUZZER_CHANNEL, 3200);
    delay(250);
    ledcWriteTone(BUZZER_CHANNEL, 0);
    #endif

    #ifdef IS_BOOSTER
    ledcWriteTone(BUZZER_CHANNEL, 2600);
    delay(150);
    ledcWriteTone(BUZZER_CHANNEL, 0);
    delay(75);
    ledcWriteTone(BUZZER_CHANNEL, 2600);
    delay(150);
    ledcWriteTone(BUZZER_CHANNEL, 0);
    delay(75);
    ledcWriteTone(BUZZER_CHANNEL, 2600);
    delay(150);
    ledcWriteTone(BUZZER_CHANNEL, 0);
    #endif


    // begin sensor SPI bus
    Serial.println("Starting SPI...");
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    //begin I2C bus
    Serial.println("Starting I2C...");
    Wire.begin(I2C_SDA, I2C_SCL, 100000);

    if (!TCAL9538Init(EXP_RESET)) {
        Serial.println(":(");
    }

    //pinMode changes need to be made here for midas mini bc of new sensors.

    //set all chip selects high (deselected)
	pinMode(E22_CS, OUTPUT);
	pinMode(MS5611_CS, OUTPUT);
    pinMode(IMU_CS_PIN, OUTPUT);
    pinMode(MMC5983_CS, OUTPUT);

	digitalWrite(MS5611_CS, HIGH);
	digitalWrite(E22_CS, HIGH);
    digitalWrite(IMU_CS_PIN, HIGH);
    digitalWrite(MMC5983_CS, HIGH);

    //configure output leds
    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_ORANGE, OUTPUT);
    pinMode(LED_RED, OUTPUT);

    gpioPinMode(PYROA_FIRE_PIN, OUTPUT);
    gpioPinMode(PYROB_FIRE_PIN, OUTPUT);
    gpioPinMode(PYROC_FIRE_PIN, OUTPUT);
    gpioPinMode(PYROD_FIRE_PIN, OUTPUT);
    gpioPinMode(PYRO_GLOBAL_ARM_PIN, OUTPUT);

    delay(200);

    // init and start threads
    begin_systems(&systems);
}

void loop()
{
}
