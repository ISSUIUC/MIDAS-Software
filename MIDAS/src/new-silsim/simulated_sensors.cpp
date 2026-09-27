#include "simulated_sensors.h"



Magnetometer MagnetometerSensor::read() {
    // read from aforementioned global instance of sensor
    uint32_t cx, cy, cz;
    double X, Y, Z;
    
    // PSEUDOCODE
	/*
		get timestamp is it time_stamp = pdTICKS_TO_MS(xTaskGetTickCount()); ??
		get sensor data from csv at timestamp
		put that in cx, cy, cz
		happy
		yay
	*/
    
    // The magnetic field values are 18-bit unsigned. The _approximate_ zero (mid) point is 2^17
    // Here we scale each field to +/- 1.0 to make it easier to convert to Gauss
    // https://github.com/sparkfun/SparkFun_MMC5983MA_Magnetometer_Arduino_Library/tree/main/examples
    double sf = (double)(1 << 17);
    X = ((double)cx - sf)/sf;
    Y = ((double)cy - sf)/sf;
    Z = ((double)cz - sf)/sf;
    
    // We multiply by 8, which is the full scale of the mag.
    // https://github.com/sparkfun/SparkFun_MMC5983MA_Magnetometer_Arduino_Library/blob/main/examples/Example4-SPI_Simple_measurement/Example4-SPI_Simple_measurement.ino
    Magnetometer reading{Y*8, -X*8, -Z*8};
    return reading;
}
