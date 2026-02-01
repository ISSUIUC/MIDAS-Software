#pragma once

#include "rocket_state.h"
#include "errors.h"

#if defined(SILSIM)
//#include "silsim/FileSink.h"
#elif defined(HILSIM)
#else
//#include "hardware/SDLog.h"
//#include "hardware/Emmc.h"
#endif




/**
 * @class LogSink
 * 
 * @brief Protocol for a sink, which is implemented as an SD card in hardware.
 */
class LogSink {
public:
    LogSink() = default;

    virtual ErrorCode init() = 0;
    virtual void write(const uint8_t* data, size_t size) = 0;
    virtual void write_meta(const uint8_t* data, size_t size) = 0;
};

void log_begin(LogSink& sink);
void log_data(LogSink& sink, RocketData& data);

template<typename... Sinks>
class MultipleLogSink : public LogSink {
public:
    MultipleLogSink() = default;

    ErrorCode init() override {
        return ErrorCode::NoError;
    };

    void write(const uint8_t* data, size_t size) override {};
};

template<typename Sink, typename... Sinks>
class MultipleLogSink<Sink, Sinks...> : public LogSink {
public:
    MultipleLogSink() = default;
    explicit MultipleLogSink(Sink sink_, Sinks... sinks_) : sink(sink_), sinks(sinks_...) { };

    ErrorCode init() override {
        ErrorCode result = sink.init();
        if (result != ErrorCode::NoError) {
            return result;
        }
        return sinks.init();
    };
    void write(const uint8_t* data, size_t size) override {
        sink.write(data, size);
        sinks.write(data, size);
    };

private:
    Sink sink;
    MultipleLogSink<Sinks...> sinks;
};

#ifndef SILSIM
#include <FS.h>
char* sdFileNamer(char* fileName, char* fileExtensionParam, FS& fs);
#endif