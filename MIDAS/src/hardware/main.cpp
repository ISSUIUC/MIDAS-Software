#include <Wire.h>
#include <SPI.h>
#include "TCAL9538.h"

#include "flight-systems/systems.h"
#include "SDLog.h"
#include "flight-systems/sensor_data.h"
#include "pins.h"

/**
 * @brief Global SD card log sink.
 *
 * Stores flight data to persistent storage throughout system operation.
 */
SDSink sink;

// #else
// MultipleLogSink<> sinks;
// #endif

/**
 * @brief Global rocket systems instance.
 *
 * Contains all subsystem controllers and shared resources used during
 * flight. The configured log sink is supplied during construction.
 */
RocketSystems systems{.log_sink = sink};

/**
 * @brief Initializes the flight computer hardware and starts all system
 *        tasks.
 *
 * This function performs the complete startup sequence:
 * - Initializes serial communication.
 * - Plays the startup tone.
 * - Initializes the SPI and I²C buses.
 * - Configures the I/O expander.
 * - Configures all GPIO pins.
 * - Initializes the flight systems.
 * - Starts the system scheduler.
 */
void setup()
{
    // Initialize the serial console for debugging.
    Serial.begin(115200);

    delay(200);

    // Configure the buzzer and play the startup tone.
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);

    // Startup beeps.
    ledcWriteTone(BUZZER_CHANNEL, 3200);
    delay(250);
    ledcWriteTone(BUZZER_CHANNEL, 0);
    delay(100);
    ledcWriteTone(BUZZER_CHANNEL, 3200);
    delay(250);
    ledcWriteTone(BUZZER_CHANNEL, 0);

    // Initialize the shared SPI bus used by onboard peripherals.
    Serial.println("Starting SPI...");
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    // Initialize the shared I²C bus.
    Serial.println("Starting I2C...");
    Wire.begin(I2C_SDA, I2C_SCL, 100000);

    // Initialize the GPIO expander.
    if (!TCAL9538Init(EXP_RESET)) {
        Serial.println(":(");
    }

    // Configure the SPI chip-select pins.
    // (MIDAS Mini may require different sensor configurations.)
    pinMode(E22_CS, OUTPUT);
    pinMode(MS5611_CS, OUTPUT);
    pinMode(IMU_CS_PIN, OUTPUT);
    pinMode(MMC5983_CS, OUTPUT);

    // Deselect all SPI devices before communication begins.
    digitalWrite(MS5611_CS, HIGH);
    digitalWrite(E22_CS, HIGH);
    digitalWrite(IMU_CS_PIN, HIGH);
    digitalWrite(MMC5983_CS, HIGH);

    // Configure the board-to-board interface.
    pinMode(B2B_EN, OUTPUT);
    pinMode(B2B_READY, INPUT);

    // Enable the board-to-board communication bus.
    digitalWrite(B2B_EN, HIGH);

    // Configure the status indicator LEDs.
    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_ORANGE, OUTPUT);
    pinMode(LED_RED, OUTPUT);

    // Configure all pyro output channels.
    for (int i = 0; i < MIDAS_NUM_PYROS; i++) {
        gpioPinMode(PYRO_PINS[i], OUTPUT);
    }

    // Configure the global pyro arm output.
    gpioPinMode(PYRO_GLOBAL_ARM_PIN, OUTPUT);

    // Allow hardware to stabilize before initialization.
    delay(200);

    // Initialize all flight systems and start their tasks.
    begin_systems(&systems);

    // Execution should not normally return from begin_systems().
    loop();
}

/**
 * @brief Default application loop.
 *
 * Once the RTOS has started, all application logic executes within
 * dedicated tasks. This function should never be reached during normal
 * operation.
 */
void loop()
{
    printf("\nHI!");
}