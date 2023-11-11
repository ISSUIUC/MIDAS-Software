/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.7 */

#ifndef PB_HILSIMPACKET_PB_H_INCLUDED
#define PB_HILSIMPACKET_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Struct definitions */
typedef struct _HILSIMPacket {
    /* High-G IMU data */
    float imu_high_ax;
    float imu_high_ay;
    float imu_high_az;
    /* Barometer data */
    float barometer_altitude;
    float barometer_temperature;
    float barometer_pressure;
    /* Low-G IMU data */
    float imu_low_ax;
    float imu_low_ay;
    float imu_low_az;
    /* Low-G lsm IMU data */
    float imu_low_lsm_ax;
    float imu_low_lsm_ay;
    float imu_low_lsm_az;
    float imu_low_lsm_gx;
    float imu_low_lsm_gy;
    float imu_low_lsm_gz;
    /* Mag data */
    float mag_x;
    float mag_y;
    float mag_z;
    /* Orientation data */
    float ornt_roll;
    float ornt_pitch;
    float ornt_yaw;
    float ornt_rollv;
    float ornt_pitchv;
    float ornt_yawv;
    float ornt_rolla;
    float ornt_pitcha;
    float ornt_yawa;
    float ornt_ax;
    float ornt_ay;
    float ornt_az;
    float ornt_gx;
    float ornt_gy;
    float ornt_gz;
    float ornt_mx;
    float ornt_my;
    float ornt_mz;
    float ornt_temp;
} HILSIMPacket;


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define HILSIMPacket_init_default                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define HILSIMPacket_init_zero                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define HILSIMPacket_imu_high_ax_tag             1
#define HILSIMPacket_imu_high_ay_tag             2
#define HILSIMPacket_imu_high_az_tag             3
#define HILSIMPacket_barometer_altitude_tag      4
#define HILSIMPacket_barometer_temperature_tag   5
#define HILSIMPacket_barometer_pressure_tag      6
#define HILSIMPacket_imu_low_ax_tag              7
#define HILSIMPacket_imu_low_ay_tag              8
#define HILSIMPacket_imu_low_az_tag              9
#define HILSIMPacket_imu_low_lsm_ax_tag          10
#define HILSIMPacket_imu_low_lsm_ay_tag          11
#define HILSIMPacket_imu_low_lsm_az_tag          12
#define HILSIMPacket_imu_low_lsm_gx_tag          13
#define HILSIMPacket_imu_low_lsm_gy_tag          14
#define HILSIMPacket_imu_low_lsm_gz_tag          15
#define HILSIMPacket_mag_x_tag                   16
#define HILSIMPacket_mag_y_tag                   17
#define HILSIMPacket_mag_z_tag                   18
#define HILSIMPacket_ornt_roll_tag               19
#define HILSIMPacket_ornt_pitch_tag              20
#define HILSIMPacket_ornt_yaw_tag                21
#define HILSIMPacket_ornt_rollv_tag              22
#define HILSIMPacket_ornt_pitchv_tag             23
#define HILSIMPacket_ornt_yawv_tag               24
#define HILSIMPacket_ornt_rolla_tag              25
#define HILSIMPacket_ornt_pitcha_tag             26
#define HILSIMPacket_ornt_yawa_tag               27
#define HILSIMPacket_ornt_ax_tag                 28
#define HILSIMPacket_ornt_ay_tag                 29
#define HILSIMPacket_ornt_az_tag                 30
#define HILSIMPacket_ornt_gx_tag                 31
#define HILSIMPacket_ornt_gy_tag                 32
#define HILSIMPacket_ornt_gz_tag                 33
#define HILSIMPacket_ornt_mx_tag                 34
#define HILSIMPacket_ornt_my_tag                 35
#define HILSIMPacket_ornt_mz_tag                 36
#define HILSIMPacket_ornt_temp_tag               37

/* Struct field encoding specification for nanopb */
#define HILSIMPacket_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, FLOAT,    imu_high_ax,       1) \
X(a, STATIC,   REQUIRED, FLOAT,    imu_high_ay,       2) \
X(a, STATIC,   REQUIRED, FLOAT,    imu_high_az,       3) \
X(a, STATIC,   REQUIRED, FLOAT,    barometer_altitude,   4) \
X(a, STATIC,   REQUIRED, FLOAT,    barometer_temperature,   5) \
X(a, STATIC,   REQUIRED, FLOAT,    barometer_pressure,   6) \
X(a, STATIC,   REQUIRED, FLOAT,    imu_low_ax,        7) \
X(a, STATIC,   REQUIRED, FLOAT,    imu_low_ay,        8) \
X(a, STATIC,   REQUIRED, FLOAT,    imu_low_az,        9) \
X(a, STATIC,   REQUIRED, FLOAT,    imu_low_lsm_ax,   10) \
X(a, STATIC,   REQUIRED, FLOAT,    imu_low_lsm_ay,   11) \
X(a, STATIC,   REQUIRED, FLOAT,    imu_low_lsm_az,   12) \
X(a, STATIC,   REQUIRED, FLOAT,    imu_low_lsm_gx,   13) \
X(a, STATIC,   REQUIRED, FLOAT,    imu_low_lsm_gy,   14) \
X(a, STATIC,   REQUIRED, FLOAT,    imu_low_lsm_gz,   15) \
X(a, STATIC,   REQUIRED, FLOAT,    mag_x,            16) \
X(a, STATIC,   REQUIRED, FLOAT,    mag_y,            17) \
X(a, STATIC,   REQUIRED, FLOAT,    mag_z,            18) \
X(a, STATIC,   REQUIRED, FLOAT,    ornt_roll,        19) \
X(a, STATIC,   REQUIRED, FLOAT,    ornt_pitch,       20) \
X(a, STATIC,   REQUIRED, FLOAT,    ornt_yaw,         21) \
X(a, STATIC,   REQUIRED, FLOAT,    ornt_rollv,       22) \
X(a, STATIC,   REQUIRED, FLOAT,    ornt_pitchv,      23) \
X(a, STATIC,   REQUIRED, FLOAT,    ornt_yawv,        24) \
X(a, STATIC,   REQUIRED, FLOAT,    ornt_rolla,       25) \
X(a, STATIC,   REQUIRED, FLOAT,    ornt_pitcha,      26) \
X(a, STATIC,   REQUIRED, FLOAT,    ornt_yawa,        27) \
X(a, STATIC,   REQUIRED, FLOAT,    ornt_ax,          28) \
X(a, STATIC,   REQUIRED, FLOAT,    ornt_ay,          29) \
X(a, STATIC,   REQUIRED, FLOAT,    ornt_az,          30) \
X(a, STATIC,   REQUIRED, FLOAT,    ornt_gx,          31) \
X(a, STATIC,   REQUIRED, FLOAT,    ornt_gy,          32) \
X(a, STATIC,   REQUIRED, FLOAT,    ornt_gz,          33) \
X(a, STATIC,   REQUIRED, FLOAT,    ornt_mx,          34) \
X(a, STATIC,   REQUIRED, FLOAT,    ornt_my,          35) \
X(a, STATIC,   REQUIRED, FLOAT,    ornt_mz,          36) \
X(a, STATIC,   REQUIRED, FLOAT,    ornt_temp,        37)
#define HILSIMPacket_CALLBACK NULL
#define HILSIMPacket_DEFAULT NULL

extern const pb_msgdesc_t HILSIMPacket_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define HILSIMPacket_fields &HILSIMPacket_msg

/* Maximum encoded size of messages (where known) */
#define HILSIMPacket_size                        207

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
