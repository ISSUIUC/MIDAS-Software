#include <Wire.h>
#include <SPI.h>

//#include "systems.h"
#include "hardware/pins.h"
//#include "hardware/Emmc.h"
//#include "hardware/SDLog.h"
#include "sensor_data.h"

/**
 * Sets the config file and then starts all the threads using the config.
 */
//MultipleLogSink<FileSink, EMMCSink> sinks;
//RocketSystems systems { .log_sink = sinks };

#include <Adafruit_BNO08x.h>
Adafruit_BNO08x bno_test(GpioAddress(1, 07));

static Vec3 quaternionToEuler(float qr, float qi, float qj, float qk, bool degrees) {
    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    Vec3 euler;
    euler.x = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));        // roll
    euler.y = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));       // yaw
    euler.z = -1 * atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));  // pitch
    return euler;
}

static Vec3 quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, bool degrees) {
    return quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k,
                             degrees);

}

static Vec3 quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, bool degrees) {
    return quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k,
                             degrees);
}

void setup() {
    //begin serial port
    Serial.begin(9600);

    delay(200);

    //begin sensor SPI bus
    Serial.println("Starting SPI...");
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    //begin I2C bus
    Serial.println("Starting I2C...");
    Wire.begin(I2C_SDA, I2C_SCL);

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

    delay(200);

    Serial.println("Delaying"); Serial.flush();
    delay(5000);
    if (!bno_test.begin_SPI(BNO086_CS, BNO086_INT)) {
        Serial.println("could not init orientation"); Serial.flush();
        while(1);
    }
    if (!bno_test.enableReport(SH2_ARVR_STABILIZED_RV, 5000)) {
        Serial.println("Could not enable stabilized remote vector");
        while(1);
    }
    Serial.println("orientation init successfully");

    while (true) {
        sh2_SensorValue_t event;
        Vec3 euler;
        if (bno_test.getSensorEvent(&event)) {
            switch (event.sensorId) {
                case SH2_ARVR_STABILIZED_RV:
                    euler = quaternionToEulerRV(&event.un.arvrStabilizedRV, true);
                case SH2_GYRO_INTEGRATED_RV:
                    euler = quaternionToEulerGI(&event.un.gyroIntegratedRV, true);
                    break;
            }
            Serial.print("yaw: ");
            Serial.print(euler.y);
            Serial.print(" pitch: ");
            Serial.print(euler.z);
            Serial.print(" roll: ");
            Serial.println(euler.x);
        }

        delay(500);
    }

//    begin_systems(&systems);
}

void loop() {

}
