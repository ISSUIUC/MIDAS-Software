#include <Arduino.h>

#include <Wire.h>
#include <SPI.h>

#include <systems.h>
#include <hal.h>

#include "sensor_data.h"
#include "log_checksum.h"
#include "global_packet.h"

#include "SDLog.h"

HILSIMPacket global_packet = HILSIMPacket_init_zero;

MultipleLogSink<SDSink> sink;
RocketSystems systems{.log_sink = sink};

DECLARE_THREAD(hilsim, void*arg) {
    int n = 0;
    // Debug kamaji output to verify if we're reading the correct packets

    while (true) {
        while (!Serial.available());
        int tag = Serial.read();

        if (tag == 1) {
            // LowGData: ax, ay, az
            Serial.readBytes(reinterpret_cast<char*>(&(systems.sensors.low_g.lowg)), sizeof(LowGData));
            // arg->rocket_data.low_g.update(lowgdata);
            Serial.print("LowG");
        }
        else if (tag == 2) {
            // HighGData: ax, ay, az
            HighGData highgdata;
            Serial.readBytes(reinterpret_cast<char*>(&(systems.sensors.high_g.highg)), sizeof(HighGData));
            // arg->rocket_data.high_g.update(highgdata);
            Serial.print("HighG");
        }
        else if (tag == 9) {
            // LowGLSM: gx, gy, gz, ax, ay, az
            LowGLSM lowglsm;
            Serial.readBytes(reinterpret_cast<char*>(&(systems.sensors.low_g_lsm.lowglsm)), sizeof(LowGLSM));
            // arg->rocket_data.low_g_lsm.update(lowglsm);
            Serial.print("LowGLSM");
        }
        else if (tag == 3) {
            // Barometer: temperature, pressure, altitude
            Barometer barometer;
            Serial.readBytes(reinterpret_cast<char*>(&(systems.sensors.barometer.barometer)), sizeof(Barometer));
            // arg->rocket_data.barometer.update(barometer);
            Serial.print("BArometer");
        }
        else if (tag == 4) {
            // Continuity: sense_pyro and pin continuity data
            Continuity continuity;
            Serial.readBytes(reinterpret_cast<char*>(&(systems.sensors.continuity.continuity)), sizeof(Continuity));
            // arg->rocket_data.continuity.update(continuity);
            Serial.print("Continuity");
        }
        else if (tag == 5) {
            // Voltage: single float value
            Voltage voltage;
            Serial.readBytes(reinterpret_cast<char*>(&(systems.sensors.voltage.voltage)), sizeof(Voltage));
            // arg->rocket_data.voltage.update(voltage);
            Serial.print("Voltage");
        }
        else if (tag == 6) {
            // GPS: latitude, longitude, altitude, speed, satellite_count, timestamp
            GPS gps;
            Serial.readBytes(reinterpret_cast<char*>(&(systems.sensors.gps.gps)), sizeof(GPS));
            // arg->rocket_data.gps.update(gps);
            Serial.print("GPS");
        }
        else if (tag == 7) {
            // Magnetometer: mx, my, mz
            Magnetometer magnetometer;
            Serial.readBytes(reinterpret_cast<char*>(&(systems.sensors.magnetometer.mag)), sizeof(Magnetometer));
            // arg->rocket_data.magnetometer.update(magnetometer);
            Serial.print("Magnetometer");
        }
        else if (tag == 8) {
            // Orientation: yaw, pitch, roll, etc.
            Orientation orientation;
            Serial.readBytes(reinterpret_cast<char*>(&(systems.sensors.orientation.orient)), sizeof(Orientation));
            // arg->rocket_data.orientation.update(orientation);
            Serial.print("Orientation");
        }
        else if (tag == 10) {
            FSMState fsm_state;
            Serial.readBytes(reinterpret_cast<char*>(&(fsm_state)), sizeof(FSMState));
            Serial.print("FSM state");
            //. We should ignore fsm state lol
        }
        else if (tag == 11) {
            // KalmanData: position, velocity, acceleration, altitude
            KalmanData kalman_data;
            Serial.readBytes(reinterpret_cast<char*>(&(kalman_data)), sizeof(KalmanData));
            // arg->rocket_data.kalman_data.update(kalman_data);

            Serial.print("kf data"); // We also ignore kf data
        }
        else if (tag == 12) {
            // PyroState: global armed state and channel data
            PyroState pyro_state;
            Serial.readBytes(reinterpret_cast<char*>(&(pyro_state)), sizeof(PyroState));
            // arg->rocket_data.pyro_state.update(pyro_state);
            Serial.print("pyro state");
        }
        else {
            // Unknown tag, handle error
            Serial.print("Error: Unknown tag received!");
            Serial.print(tag);
        }
    
        Serial.println("Read line");
        // Print fsm state
        Serial.flush();
    }
}

void setup(){
    Serial.begin(115200);
    while (!Serial);
    while (!Serial.available()) {}
    while (Serial.read() != 33) ;
    char magic[] = {69, 110, 117, 109, 99, 108, 97, 119, 0};
    Serial.print(magic);
    Serial.print('\n');
    Serial.print(LOG_CHECKSUM);
    Serial.print('\n');
    Serial.flush();

    //begin sensor SPI bus
    // Serial.println("Starting SPI...");
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    //begin I2C bus
    // Serial.println("Starting I2C...");
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
    begin_systems(&systems);

}

void loop(){}
