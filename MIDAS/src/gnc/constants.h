// constants
const float pi = 3.14159268;
const float a = 343.0;                // (m/s) speed of sound
const float rho = 1.225;              // average air density
const float r = 0.0396;              // (m)
const float height_full = 3.0259;      // (m) height of rocket Full Stage
const float height_sustainer = 1.5021; // (m) height of rocket Sustainer
const float mass_sustainer_dry = 3.61;        // (kg) Sustainer Dry Mass
const float mass_sustainer_wet = 4.68;        // (kg) Sustainer Wet Mass
const float mass_booster_dry = 3.76; // (kg) Booster dry mass
const float mass_booster_wet = 5.91; // (kg) Booster wet mass
const float mass_full = mass_sustainer_wet+mass_booster_wet; // (kg) Total mass wet
const float mass_first_burnout = mass_booster_dry+mass_sustainer_wet;// (kg) Total mass after first stage burnout
const float mass_second_burnout = mass_booster_dry+mass_sustainer_dry;// (kg) Total mass after first stage burnout
const float gravity_ms2 = 9.81;           // (m/s^2) accel due to gravity
