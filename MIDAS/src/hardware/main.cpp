#include <Wire.h>
#include <SPI.h>
#include "TCAL9539.h"

#include "systems.h"
#include "hardware/pins.h"
#include "hardware/Emmc.h"
#include "hardware/SDLog.h"
#include "sensor_data.h"


void i2cscanner() {
    byte error, address;
    int nDevices;
 
  Serial.println("Scanning...");
 
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}
/**
 * Sets the config file and then starts all the threads using the config.
 */

#ifdef IS_SUSTAINER
// MultipleLogSink<EMMCSink> sinks;
MultipleLogSink<SDSink> sinks;
#else
MultipleLogSink<> sinks;
#endif
RocketSystems systems { .log_sink = sinks };
/**
 * @brief Sets up pinmodes for all sensors and starts threads
*/

void setup() {
    //begin serial port
    Serial.begin(9600);

   while (!Serial);

    delay(200);

    // begin sensor SPI bus
    Serial.println("Starting SPI...");
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    //begin I2C bus
    Serial.println("Starting I2C...");
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire1.begin(PYRO_SDA, PYRO_SCL);

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

    gpioPinMode(PYROA_FIRE_PIN, OUTPUT, 1);
    gpioPinMode(PYROB_FIRE_PIN, OUTPUT, 1);
    gpioPinMode(PYROC_FIRE_PIN, OUTPUT, 1);
    gpioPinMode(PYROD_FIRE_PIN, OUTPUT, 1);
    gpioPinMode(PYRO_GLOBAL_ARM_PIN, OUTPUT, 1);

    delay(200);

    if (!TCAL9539Init(GPIO_RESET_PIN)) {
			Serial.println("Failed to initialize TCAL9539!");
			// while(1){ };
		}

    //init and start threads
    begin_systems(&systems);
}

void loop() {

}
