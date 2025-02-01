#include <Arduino.h>

#include "buzzer_backend.h"

#include "pins.h"

void BuzzerBackend::init() {
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
}

void BuzzerBackend::play_tone(uint32_t frequency) {
    ledcWriteTone(BUZZER_CHANNEL, frequency);
}
