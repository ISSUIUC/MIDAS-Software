#include<Arduino.h>

namespace Pins {
    static constexpr uint8_t BOOT = 0;
    static constexpr uint8_t E22_BUSY_0 = 0;
    static constexpr uint8_t SPI_MISO_0 = 0;
    static constexpr uint8_t SPI_MOSI_0 = 0;
    static constexpr uint8_t E22_CS_0 = 0;
    static constexpr uint8_t SPI_SCK_0 = 0;
    static constexpr uint8_t E22_DIO1_0 = 0;
    static constexpr uint8_t D22_RXEN_0 = 0;
    static constexpr uint8_t E22_TXEN_0 = 0;
    static constexpr uint8_t E22_RESET_0 = 0;
    static constexpr uint8_t SPI_MISO_1 = 0;
    static constexpr uint8_t SPI_MOSI_1 = 0;
    static constexpr uint8_t LED_BLUE = 0;
    static constexpr uint8_t LED_GREEN = 0;
    static constexpr uint8_t LED_ORANGE = 0;
    static constexpr uint8_t LED_RED = 0;
    static constexpr uint8_t SPI_SCK_1 = 0;
    static constexpr uint8_t E22_CS_1 = 0;
    static constexpr uint8_t E22_BUSY_1 = 0;
    static constexpr uint8_t E22_RXEN_1 = 0;
    static constexpr uint8_t E22_RXEN_1 = 0;
    static constexpr uint8_t E22_DIO3_1 = 0;
    static constexpr uint8_t E22_DIO2_1 = 0;
    static constexpr uint8_t E22_DIO2_0 = 0;
}


void setup() {
    Serial.begin(9600);
    pinMode(12, OUTPUT);
    pinMode(13, OUTPUT);
    pinMode(14, OUTPUT);
    pinMode(15, OUTPUT);
}

void loop() {
    Serial.println("HELLO");
    delay(500);
    digitalWrite(12, HIGH);
    digitalWrite(13, HIGH);
    digitalWrite(14, HIGH);
    digitalWrite(15, HIGH);
    delay(500);
    digitalWrite(12, LOW);
    digitalWrite(13, LOW);
    digitalWrite(14, LOW);
    digitalWrite(15, LOW);
}