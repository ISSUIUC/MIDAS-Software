#include <Wire.h>
#include <SPI.h>
#include "TCAL9539.h"
#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xf4,0x12,0xfa,0x74,0x84,0xbc};
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
MultipleLogSink<> sinks;
// #else
// MultipleLogSink<> sinks;
// #endif
RocketSystems systems{.log_sink = sinks};
/**
 * @brief Sets up pinmodes for all sensors and starts threads
 */

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//   Serial.print("\r\nLast Packet Send Status:\t");
if(status != ESP_NOW_SEND_SUCCESS) Serial.println("Delivery fail");
//   Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
esp_now_peer_info_t peerInfo;

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
    Wire1.begin(PYRO_SDA, PYRO_SCL, 400000);

    if (!TCAL9539Init(GPIO_RESET)) {
        Serial.println(":(");
    }

    //set all chip selects high (deselected)
    pinMode(LSM6DS3_CS, OUTPUT);
	pinMode(KX134_CS, OUTPUT);
	pinMode(ADXL355_CS, OUTPUT);
	pinMode(LIS3MDL_CS, OUTPUT);
	pinMode(BNO086_CS, OUTPUT);
	pinMode(BNO086_RESET, OUTPUT);
	pinMode(CAN_CS, OUTPUT);
	pinMode(E22_CS, OUTPUT);
	pinMode(MS5611_CS, OUTPUT);

	digitalWrite(MS5611_CS, HIGH);
	digitalWrite(LSM6DS3_CS, HIGH);
	digitalWrite(KX134_CS, HIGH);
	digitalWrite(ADXL355_CS, HIGH);
	digitalWrite(LIS3MDL_CS, HIGH);
	digitalWrite(BNO086_CS, HIGH);
	digitalWrite(CAN_CS, HIGH);
	digitalWrite(E22_CS, HIGH);
    //configure output leds
    gpioPinMode(LED_BLUE, OUTPUT);
    gpioPinMode(LED_GREEN, OUTPUT);
    gpioPinMode(LED_ORANGE, OUTPUT);
    gpioPinMode(LED_RED, OUTPUT);

    gpioPinMode(PYROA_FIRE_PIN, OUTPUT);
    gpioPinMode(PYROB_FIRE_PIN, OUTPUT);
    gpioPinMode(PYROC_FIRE_PIN, OUTPUT);
    gpioPinMode(PYROD_FIRE_PIN, OUTPUT);
    gpioPinMode(PYRO_GLOBAL_ARM_PIN, OUTPUT);

    WiFi.mode(WIFI_STA);
    if(esp_now_init() != ESP_OK) {
        Serial.println("Error initing ESP NOW (protocol for talking with stepper board)");
    }
    esp_now_register_send_cb(OnDataSent);

    // Register peer
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add esp now peer");
    }

    delay(200);

    // init and start threads
    begin_systems(&systems);
}

void loop()
{
}
