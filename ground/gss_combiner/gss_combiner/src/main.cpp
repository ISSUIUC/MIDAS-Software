#include <Arduino.h>



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println('{"type":"data","value":{"gps_lat":0,"gps_long":0,"gps_alt":100,"KX_IMU_ax":0.31641,"KX_IMU_ay":0.09766,"KX_IMU_az":1.02734,"LOW_G_LSM_ax":0,"LOW_G_LSM_ay":0,"LOW_G_LSM_az":0,"IMU_gx":1.375,"IMU_gy":-0.125,"IMU_gz":2.125,"IMU_mx":-0.68872,"IMU_my":0.75269,"IMU_mz":-2.66418,"FSM_state":0,"sign":"KD9ZMJ","RSSI":-128,"Voltage":0.375,"Continuity1":0,"Continuity2":1.875,"Continuity3":0,"Continuity4":0,"Pyro1":0,"Pyro2":0,"Pyro3":0,"Pyro4":0,"Pyro1Firing":0,"Pyro2Firing":0,"Pyro3Firing":0,"Pyro4Firing":0,"TelemLatency":260,"LogLatency":0,"is_booster":0,"sense_pyro":0,"BNO_YAW":0,"BNO_PITCH":0,"BNO_ROLL":0,"TEMP":28.48828,"pressure":1000.375}}');
}