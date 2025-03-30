#include "sensors.h"
#include "pins.h"

#include <PL_ADXL355.h>
#include <Arduino_LSM6DS3.h>
#include <SparkFun_Qwiic_KX13X.h>
#include <MS5611.h>
#include <ads7138-q1.h>
#include <Adafruit_LIS3MDL.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <TCAL9539.h>
#include "E22.h"

// Change to 434.0 or other frequency, must match RX's freq!
#ifdef IS_BOOSTER
#define TX_FREQ 425.15
#else
#define TX_FREQ 421.15
#endif

// Which b2b communication we should use
#define B2B_I2C
// #define B2B_CAN

#if defined(B2B_I2C) && defined(B2B_CAN)
#error "B2B can only use one option of B2B_I2C, or B2B_CAN"
#elif !defined(B2B_I2C) && !defined(B2B_CAN)
#error "At least one B2B_I2C or B2B_CAN must be defined"
#endif

#if defined(B2B_CAN)
#error "you've been baited lmfao"
#endif

PL::ADXL355 low_g(ADXL355_CS);
LSM6DS3Class low_g_lsm(SPI, LSM6DS3_CS, 46);
QwiicKX134 KX;
MS5611 MS(MS5611_CS);
Adafruit_LIS3MDL LIS3MDL;
SFE_UBLOX_GNSS ublox;
SX1268 lora(SPI, E22_CS, E22_BUSY, E22_DI01, E22_RXEN, E22_RESET);

ErrorCode init_low_g() {
    low_g.begin();
    low_g.setRange(PL::ADXL355_Range::range2g);
    low_g.setOutputDataRate(PL::ADXL355_OutputDataRate::odr1000);
    // todo set low pass filter frequency to 250hx
    low_g.enableMeasurement();
    return ErrorCode::NoError;
}

LowGData HwImpl::read_low_g() {
    auto data = low_g.getAccelerations();
    return { data.x, data.y, data.z };
}

ErrorCode init_low_g_lsm() {
    if (!low_g_lsm.begin()) {
        return ErrorCode::GyroCouldNotBeInitialized;
    }
    return ErrorCode::NoError;
}

LowGLSMData HwImpl::read_low_g_lsm() {
    LowGLSMData result;
    low_g_lsm.readAcceleration(result.ax, result.ay, result.az);
    low_g_lsm.readGyroscope(result.gx, result.gy, result.gz);
    return result;
}

ErrorCode init_high_g() {
    KX.beginSPI(KX134_CS);
    if (!KX.initialize(DEFAULT_SETTINGS)) {
        return ErrorCode::HighGCouldNotBeInitialized;
    }

    if(!KX.setOutputDataRate(0xb)) {
        return ErrorCode::HighGCouldNotUpdateDataRate;
    }

    KX.setRange(3);
    return ErrorCode::NoError;
}

HighGData HwImpl::read_high_g() {
    auto data = KX.getAccelData();
    return { data.xData, data.yData, data.zData };
}

ErrorCode init_barometer() {
    MS.init();

    return ErrorCode::NoError;
}

BarometerData HwImpl::read_barometer() {
    MS.read(12);

    /*
     * TODO: Switch to latest version of library (0.3.9) when we get hardware to verify
     * Equation derived from https://en.wikipedia.org/wiki/Atmospheric_pressure#Altitude_variation
    */
    float pressure = static_cast<float>(MS.getPressure() * 0.01 + 26.03); // getPressure is in milibars so it's milibars * 0.01?
    float temperature = static_cast<float>(MS.getTemperature() * 0.01); // Celcius
    float altitude = static_cast<float>(-log(pressure * 0.000987) * (temperature + 273.15) * 29.254);

    return { temperature, pressure, altitude };
}

ErrorCode init_continuity() {
    // ADS7138Init();              // Ask ADS to init the pins, we still need to get the device to actually read
    // Configure the INA745b        MODE:ContTCV  VBUS:1052us  VSEN:1052us  TCT:1052us   AVG:128
    constexpr uint16_t INA_config = (0xF << 12) | (0x5 << 9) | (0x5 << 6) | (0x5 << 3) | (0x4);

    Wire1.beginTransmission(0x41);
    Wire1.write(0x1);

    Wire1.write(((INA_config >> 8) & 0xFF));
    Wire1.write(((INA_config >> 0) & 0xFF));

    if(Wire1.endTransmission()){
        Serial.println("Pyro PWR I2C Error");
        return ErrorCode::ContinuityCouldNotBeInitialized;
    }

    Serial.println("Pyro PWR monitor configured");

    return ErrorCode::NoError;
}

int read_pwr_monitor_register(int address, int reg, int bytes) {
    Wire1.beginTransmission(address); // I2C Address 0x41 is pyro pwr monitor
    Wire1.write(reg);
    if(Wire1.endTransmission()){
        Serial.println("I2C Error");
    }

    Wire1.requestFrom(address, bytes);
    int val = 0;

    for(int i = 0; i < bytes; i++){
        int v = Wire1.read();
        if(v == -1) Serial.println("I2C Read Error");
        val = (val << 8) | v;
    }

    return val;
}

ContinuityData HwImpl::read_continuity() {
    ContinuityData continuity;
    //ADC reference voltage is 3.3, returns 12 bit value

    // MIDAS 2.1 rev A ADC sense fix:
    int16_t current = read_pwr_monitor_register(0x41, 0x7, 2);
    int voltage = read_pwr_monitor_register(0x41, 0x5, 2);

    float voltage_normalized = voltage * 3.125 / 1000.0; // V

    float continuous_channels = 0.0;
    float absolute_current = 0.0;
    float expected_current = 0.0;

    if(voltage_normalized > 1) {
        absolute_current = current * 1.2 / 1000.0;

        expected_current = (voltage_normalized - 0.2) / 470.0; // Account for diode voltage drop
        continuous_channels = absolute_current / expected_current;
    }

    // We don't have the granularity to determine individual voltages, so all of them will give the # of continuous channels
    continuity.pins[0] = continuous_channels; // Number of continuous channels on the pyro bus
    continuity.pins[1] = absolute_current;    // The absolute current running through the pyro bus
    continuity.pins[2] = expected_current;    // Calculated expected current based on current pyro bus voltage
    continuity.pins[3] = voltage_normalized;  // Pyro bus voltage

    // Serial.println(continuous_channels);
    return continuity;
}

VoltageData HwImpl::read_voltage() {
    VoltageData v_battery;
    int voltage = read_pwr_monitor_register(0x44, 0x5, 2);
    int16_t current = read_pwr_monitor_register(0x44, 0x7, 2);

    float voltage_normalized = voltage * 3.125 / 1000.0;
    float absolute_current = current * 1.2 / 1000.0;

    // Serial.print("Voltage: ");
    // Serial.println(voltage_normalized);
    // Serial.print("Current: ");
    // Serial.println(current);

    v_battery.voltage = voltage_normalized;
    v_battery.current = absolute_current;
//    Serial.print("Raw voltage reading: ");
//    Serial.print(v_battery.voltage);
//    Serial.println("");
    //* 3.3f / 4095.f / VOLTAGE_DIVIDER;
    return v_battery;
}

ErrorCode init_magnetometer() {
    if (!LIS3MDL.begin_SPI(LIS3MDL_CS)) {                     // Checks if sensor is connected
        return ErrorCode::MagnetometerCouldNotBeInitialized;
    }
    LIS3MDL.setOperationMode(LIS3MDL_CONTINUOUSMODE);         // Reading continuously, instead of single-shot or off
    LIS3MDL.setDataRate(LIS3MDL_DATARATE_155_HZ);
    LIS3MDL.setRange(LIS3MDL_RANGE_4_GAUSS);                  // Earth's magnetic field is 1/2 gauss, can detect high current
    return ErrorCode::NoError;
}

MagnetometerData HwImpl::read_magnetometer() {
    LIS3MDL.read();

    float mx = LIS3MDL.x_gauss;
    float my = LIS3MDL.y_gauss;
    float mz = LIS3MDL.z_gauss;
    MagnetometerData reading { mx, my, mz };
    return reading;
}

ErrorCode init_gps() {
    if (!ublox.begin()) {
        return ErrorCode::GPSCouldNotBeInitialized;
    }

    ublox.setDynamicModel(DYN_MODEL_AIRBORNE4g);
    ublox.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA); //Set the I2C port to output both NMEA and UBX messages
    // Set the measurment rate faster than one HZ if necessary
    // ublox.setMeasurementRate(100);
    ublox.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

    //This will pipe all NMEA sentences to the serial port so we can see them

    return ErrorCode::NoError;
}

bool HwImpl::is_gps_ready() {
    return ublox.getPVT();
}

GPSData HwImpl::read_gps() {
    return {
        ublox.getLatitude(),
        ublox.getLongitude(),
        (float) ublox.getAltitude() / 1000.f,
        (float) ublox.getGroundSpeed() / 1000.f,
        ublox.getFixType(),
        ublox.getUnixEpoch()
    };
}

static GpioAddress LED_pins[4] = {
    LED_BLUE,
    LED_RED,
    LED_ORANGE,
    LED_GREEN
};

void HwImpl::set_led(LED which, bool value) {
    gpioDigitalWrite(LED_pins[(int) which], value);
}

ErrorCode init_lora() {
    if (lora.setup() != SX1268Error::NoError) {
        return ErrorCode::LoraCouldNotBeInitialized;
    }
    if (lora.set_modulation_params(8, LORA_BW_250, LORA_CR_4_8, false) != SX1268Error::NoError) {
        return ErrorCode::LoraCommunicationFailed;
    }
    if (lora.set_frequency((uint32_t) (TX_FREQ * 1e6)) != SX1268Error::NoError) {
        return ErrorCode::LoraCommunicationFailed;
    }
    if (lora.set_tx_power(22) != SX1268Error::NoError) {
        return ErrorCode::LoraCommunicationFailed;
    }

    return ErrorCode::NoError;
}

void HwImpl::transmit_bytes(uint8_t* data, size_t count) {
    SX1268Error result = lora.send(data, count);
    if (result != SX1268Error::NoError) {
        Serial.print("Lora TX error ");
        Serial.println((int) result);
        // Reinit the lora
        init_lora();
    }
}

bool HwImpl::receive_bytes(uint8_t* memory, size_t count, int wait_milliseconds) {
    SX1268Error result = lora.recv((uint8_t*) write, count, wait_milliseconds);
    if (result == SX1268Error::NoError) {
        return true;
    } else if (result == SX1268Error::RxTimeout) {
        return false;
    } else {
        Serial.print("Lora error on rx ");
        Serial.println((int) result);

        // Reinit the lora
        init_lora();
        return false;
    }
}

bool error_is_failure(GpioError error_code) {
    return error_code != GpioError::NoError;
}

ErrorCode init_pyro() {
    bool has_failed_gpio_init = false;

    // global arm
    has_failed_gpio_init |= error_is_failure(gpioPinMode(PYRO_GLOBAL_ARM_PIN, OUTPUT));

    // fire pins
    has_failed_gpio_init |= error_is_failure(gpioPinMode(PYROA_FIRE_PIN, OUTPUT));
    has_failed_gpio_init |= error_is_failure(gpioPinMode(PYROB_FIRE_PIN, OUTPUT));
    has_failed_gpio_init |= error_is_failure(gpioPinMode(PYROC_FIRE_PIN, OUTPUT));
    has_failed_gpio_init |= error_is_failure(gpioPinMode(PYROD_FIRE_PIN, OUTPUT));

//    if (has_failed_gpio_init) {
//        return ErrorCode::PyroGPIOCouldNotBeInitialized;
//    } else {
    return ErrorCode::NoError;   // GPIO Driver always claimes it errored even when it doesn't.
//    }
}

void HwImpl::set_global_arm(bool to_high) {
    gpioDigitalWrite(PYRO_GLOBAL_ARM_PIN, to_high);
}

void HwImpl::set_pin_firing(Channel which, bool to_high) {
    GpioAddress address = PYROA_FIRE_PIN;
    switch (which) {
        case Channel::A:
            address = PYROA_FIRE_PIN;
            break;
        case Channel::B:
            address = PYROB_FIRE_PIN;
            break;
        case Channel::C:
            address = PYROC_FIRE_PIN;
            break;
        case Channel::D:
            address = PYROD_FIRE_PIN;
            break;
    }
    gpioDigitalWrite(address, to_high);
}

enum CameraCommand {
    CAMERA1_OFF = 0,
    CAMERA1_ON = 1,
    CAMERA2_OFF = 2,
    CAMERA2_ON = 3,
    VTX_OFF = 4,
    VTX_ON = 5,
    MUX_1 = 6,
    MUX_2 = 7
};

void transmit_command(CameraCommand command) {
#ifdef B2B_I2C
    Wire.beginTransmission(0x69); // 0x69 --> Camera board i2c address
    Wire.write((uint8_t) command);
    if (Wire.endTransmission()) {
        Serial.println("Camera B2B i2c write error");
    }
#else
    #error "lmao"
#endif
}

void HwImpl::set_camera_on(Camera which, bool on) {
    CameraCommand command;
    if (which == Camera::Side) {
        command = on ? CameraCommand::CAMERA1_ON : CameraCommand::CAMERA1_OFF;
    } else {
        command = on ? CameraCommand::CAMERA2_ON : CameraCommand::CAMERA2_OFF;
    }
    transmit_command(command);
}

void HwImpl::set_camera_source(Camera which) {
    // it's apparently backwards for some reason?
    CameraCommand command = which == Camera::Side ? CameraCommand::MUX_2 : CameraCommand::MUX_1;
    transmit_command(command);
}

void HwImpl::set_video_transmit(bool on) {
    CameraCommand command = on ? CameraCommand::VTX_ON : CameraCommand::VTX_OFF;
    transmit_command(command);
}

uint8_t HwImpl::get_camera_state() {
#ifdef B2B_I2C
    Wire.requestFrom(0x69, 1);
    uint8_t res = Wire.read();
    return res;
#else
    #error "lmao"
#endif
}

#define TRY(fn) do { ErrorCode code_ = fn(); if (code_ != ErrorCode::NoError) { return code_; } } while (0)

ErrorCode init_orientation();

ErrorCode HwImpl::init_all() {
    TRY(init_low_g);
    TRY(init_low_g_lsm);
    TRY(init_high_g);
    TRY(init_barometer);
    TRY(init_continuity);
//    TRY(init_voltage);
    TRY(init_orientation);
    TRY(init_magnetometer);
    TRY(init_gps);
    TRY(init_pyro);
    TRY(init_lora);
    return ErrorCode::NoError;
}
