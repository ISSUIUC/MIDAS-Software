#include "../sensor_data.h"
#include "../data_logging.h"
#include <vector>

void process_row(...) {
    // paramter = row

    /**
     * LowG
     * HighG
     * Baro
     * Cont
     * Volt
     * GPS
     * Mag
     * Orient
     * LogGLSM
     * FSM
     * Kalman
     * Pyro
     */

    LowGData low_g;
    low_g.ax = line["accel_x"];
    low_g.ay = line["accel_y"];
    low_g.az = line["accel_z"];

    HighGData high_g;
    high_g.ax = line["accel_x"];
    high_g.ay = line["accel_y"];
    high_g.az = line["accel_z"];
    
    BaroData baro;
    baro.alt = line['baro_alt'];
    


    Continuity cont;
    Voltage volt;
    GPS gps;
    Mag mag;
    Orient orient;
    orient.angposx = line["ang_pos_x"];
    orient.angposy = line["ang_pos_y"];
    orient.angpzs = line["ang_pos_z"];
    orient.angvelx = line["ang_vel_x"];
    orient.angvely = line["ang_vel_y"];
    orient.angvelz = line["ang_vel_z"];
    orient.angaccx = line["ang_accel_x"];
    orient.angaccy = line["ang_accel_y"];
    orient.angaccz = line["ang_accel_z"];
    orient.accx = line["accel_x"];
    orient.accy = line["accel_y"];
    orient.accz = line["accel_z"];
    orient.imugyrox = line["imu_gyro_x"];
    orient.imugyroy = line["imu_gyro_y"];
    orient.imugyroz = line["imu_gyro_z"];
    orient.alpha = line["alpha"];
    

    LogGLSM log;
    log.imugyrox = line["imu_gyro_x"];
    log.imugyroy = line["imu_gyro_y"];
    log.imugyroz = line["imu_gyro_z"];
    log.ax = line["accel_x"];
    log.ay = line["accel_y"];
    log.az = line["accel_z"];
    

    KalmanData kalman;
    kalman.position.px = line["kalman_pos_x"];
    kalman.position.py = line["kalman_pos_y"];
    kalman.position.pz = line["kalman_pos_z"];
    kalman.velocity.vx = line["kalman_vel_x"];
    kalman.velocity.vy = line["kalman_vel_y"];
    kalman.velocity.vz = line["kalman_vel_z"];
    kalman.acceleration.ax = line["kalman_accel_x"];
    kalman.acceleration.ay = line["kalman_accel_y"];
    kalman.acceleration.az = line["kalman_accel_z"];
    
    
}

int main(int arc, char** argv) {
    /**
     * 1. open the file
     * 2. loop through all of the rows
     * 3. process each row
     * 4. with processed data, write to file
     */


}


