#pragma oncegit


struct LowGData {
    float gx = 0;
    float gy = 0;
    float gz = 0;
};

struct HighGData {
    float gx = 0;
    float gy = 0;
    float gz = 0;
};

struct Barometer {
    float temperature = 0;
    float pressure = 0;
};

struct Continuity {
    bool is_continious = false;
};

struct Voltage {
    float voltage = 0;
};

struct GPS {
    float latitude = 0;
    float longitudinal = 0;
    float altitude = 0;
    float satalite_count = 0;
};

struct Magnetometer {
    float mx = 0;
    float my = 0;
    float mz = 0;
};

struct Orientation {
    float yaw = 0;
    float pitch = 0;
    float roll = 0;

    VelocityData orientation_velocity;
    AccelerationData orientation_acceleration;

    AccelerationData lienar_acceleration;

    float gx = 0, gy = 0, gz = 0;

    Magnetometer magnetometer;

    float temperature;
};

struct PositionData {
    float px = 0; 
    float py = 0; 
    float pz = 0;
};

struct VelocityData {
    float vx = 0; 
    float vy = 0; 
    float vz = 0;
};

struct AccelerationData {
    float ax =0 ; 
    float ay = 0; 
    float az = 0;
};

struct KalmanData {
    PositionData position;
    VelocityData velocity;
    AccelerationData acceleration;

    float altitude;

    clock_t timeStamp_state = 0;
};

struct Pyro {
    bool is_active = false;
};

