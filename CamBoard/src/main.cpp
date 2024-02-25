#include <Arduino.h>

#define LED_TEST
// #define REGULATOR_TEST

#define BLUE_LED_PIN 37
#define REGULATOR_PIN 9

void setup() {
  // put your setup code here, to run once:
  // Serial.begin(9600);
  // while(!Serial);

  // Serial.println("Beginning cam board test setup");
  #ifdef LED_TEST
    pinMode(BLUE_LED_PIN, OUTPUT); // Set BLUE LED to output
    // Serial.println("Set LED pinmode");
  #endif

  #ifdef REGULATOR_TEST
    pinMode(REGULATOR_PIN, OUTPUT); // Set 9V regulator to output
    Serial.println("Set Regulator pinmode");
  #endif

  // Serial.println("cam board test setup complete\n");
}

void loop() {
  // put your main code here, to run repeatedly:

  #ifdef REGULATOR_TEST
    Serial.println("Regulator set high");
    digitalWrite(REGULATOR_PIN, HIGH);
  #endif

  #ifdef LED_TEST
    // Flash blue LED at 1hz (ish)
    // Serial.println("LED test Loop high");
    digitalWrite(BLUE_LED_PIN, HIGH);
    delay(500);
    // Serial.println("LED test Loop low");
    digitalWrite(BLUE_LED_PIN, LOW);
    delay(500);
  #endif

  delay(10);
}