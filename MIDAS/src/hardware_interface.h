#pragma once
// the only thing #ifdef SILSIM we need, theoretically
#ifdef SILSIM
#include "silsim/emulation.h"
#else
#include <Arduino.h>
#include "FreeRTOSConfig.h"
#endif

#include "errors.h"
#include "sensor_data.h"
#include "led.h"

/**
 * @brief Delays the running thread.
 * @param millis The time to delay in milliseconds.
*/
#define THREAD_SLEEP(millis) vTaskDelay(pdMS_TO_TICKS(millis))

template<typename T>
struct ISensor {
protected:
    ~ISensor() = default;

public:
    virtual ErrorCode init() = 0;
    virtual T read() = 0;
};

/**
 * @class ILogSink
 *
 * @brief Protocol for a sink, which is implemented as an SD card in hardware.
 */
class ILogSink {
protected:
    ~ILogSink() = default;

public:
    ILogSink() = default;

    virtual ErrorCode init() = 0;
    virtual void write(const uint8_t* data, size_t size) = 0;
};

template<typename... Sinks>
class MultipleLogSink final: public ILogSink {
public:
    MultipleLogSink() = default;

    ErrorCode init() override {
        return ErrorCode::NoError;
    }

    void write(const uint8_t* data, size_t size) override {}
};

template<typename Sink, typename... Sinks>
class MultipleLogSink<Sink, Sinks...>final: public ILogSink {
public:
    MultipleLogSink() = default;
    explicit MultipleLogSink(Sink sink_, Sinks... sinks_) : sink(sink_), sinks(sinks_...) { };

    ErrorCode init() override {
        ErrorCode result = sink.init();
        if (result != ErrorCode::NoError) {
            return result;
        }
        return sinks.init();
    }

    void write(const uint8_t* data, size_t size) override {
        sink.write(data, size);
        sinks.write(data, size);
    }

private:
    Sink sink;
    MultipleLogSink<Sinks...> sinks;
};

/**
 * @class TelemetryBackend
 *
 * @brief Class that wraps the Telemetry functions
*/
class ITelemetryBackend {
protected:
    ~ITelemetryBackend() = default;

    virtual void send_bytes(const uint8_t* data, size_t length) = 0;
    virtual bool recv_bytes(uint8_t* data, size_t length, int wait_milliseconds) = 0;

public:
    virtual ErrorCode init() = 0;

    virtual int8_t getRecentRssi();
    virtual void setFrequency(float frequency);

    /**
     * @brief This function transmits data from the struct provided as
     * the parameter (data collected from sensor suite) to the
     * ground station. The function also switches to a new commanded
     * frequency based on a previously received command and waits for
     * a response from the ground station.
     *
     * @param data: struct of data from the sensor suite to be transmitted to the ground station.
     *
     * @return void
     */
    template<typename T>
    void send(const T& data) {
        send_bytes(reinterpret_cast<const uint8_t*>(&data), sizeof(T));
    }

    /**
     * @brief Reads message from the LoRa
     *
     * @param write The buffer to write the data to
     * @param wait_milliseconds How long to wait for data if any is available
     *
     * @return bool indicating a successful read and write to buffer
    */
    template<typename T>
    bool read(T* write, int wait_milliseconds) {
        return recv_bytes(reinterpret_cast<uint8_t*>(write), sizeof(T), wait_milliseconds);
    }
};

class IBuzzerBackend {
protected:
    ~IBuzzerBackend() = default;

public:
    virtual void init() = 0;
    virtual void play_tone(uint32_t frequency) = 0;
};

class ILedBackend {
protected:
    ~ILedBackend() = default;

public:
    virtual void set(LED led, bool on) = 0;
};

class IPyroBackend {
protected:
    ~IPyroBackend() = default;

public:
    virtual ErrorCode init() = 0;
    virtual void arm_all() = 0;
    virtual void fire_channel(int channel) = 0;
};

/**
 * @struct Sensors
 *
 * @brief holds all interfaces for all sensors on MIDAS
*/
struct Sensors {
    ISensor<LowGData>& low_g;
    ISensor<LowGLSMData>& low_g_lsm;
    ISensor<HighGData>& high_g;
    ISensor<BarometerData>& barometer;
    ISensor<ContinuityData>& continuity;
    ISensor<VoltageData>& voltage;
    ISensor<OrientationData>& orientation;
    ISensor<MagnetometerData>& magnetometer;
    ISensor<GPSData>& gps;
    ILogSink& sink;
    ITelemetryBackend& telemetry;
    IBuzzerBackend& buzzer;
    ILedBackend& led;
    IPyroBackend& pyro;
};
