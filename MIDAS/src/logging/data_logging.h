#pragma once

#include "flight-systems/rocket_state.h"
#include <util/errors.h>

#if defined(SILSIM)
//#include "silsim/FileSink.h"
#elif defined(HILSIM)
#else
//#include "hardware/SDLog.h"
#endif

/**
 * @class LogSink
 *
 * @brief Abstract interface for flight data logging backends.
 *
 * A LogSink provides a common interface for writing binary flight logs and
 * associated metadata regardless of the underlying storage medium. Hardware
 * implementations typically write to onboard flash or an SD card, while
 * simulation builds may implement alternative sinks.
 */
class LogSink {
public:
    LogSink() = default;

    bool failed_wr = false; ///< True if a binary log write has failed.
    bool failed_mr = false; ///< True if a metadata write has failed.

    /**
     * @brief Initializes the logging backend.
     *
     * Opens any required storage devices, creates log files, and prepares the
     * sink for subsequent write operations.
     *
     * @return ErrorCode indicating whether initialization succeeded.
     */
    virtual ErrorCode init() = 0;

    /**
     * @brief Writes binary flight data to the log.
     *
     * @param data Pointer to the byte buffer to write.
     * @param size Number of bytes to write.
     */
    virtual void write(const uint8_t* data, size_t size) = 0;

    /**
     * @brief Writes metadata to the metadata log.
     *
     * Metadata typically consists of important flight events or summary values
     * that can be quickly parsed without processing the entire binary log.
     *
     * @param data Pointer to the metadata buffer.
     * @param size Number of bytes in the metadata buffer.
     */
    virtual void write_meta(const uint8_t* data, size_t size) = 0;

    uint16_t current_file_no = 0;   ///< Current flight log file number.
    char active_bin_name[20] = {0}; ///< Name of the active binary log file.
    char active_meta_name[20] = {0};///< Name of the active metadata file.
};

/**
 * @brief Initializes the flight logging system.
 *
 * Performs any required setup before flight data begins being recorded.
 *
 * @param sink Logging backend used for storage.
 */
void log_begin(LogSink& sink);

/**
 * @brief Logs a complete rocket telemetry packet.
 *
 * Serializes and writes a RocketData packet to the configured logging backend.
 *
 * @param sink Logging backend used for storage.
 * @param data Rocket telemetry packet to log.
 */
void log_data(LogSink& sink, RocketData& data);

template<typename... Sinks>
class MultipleLogSink : public LogSink {
public:
    MultipleLogSink() = default;

    /**
     * @brief Initializes the logging sink.
     *
     * Base case for the recursive MultipleLogSink template. Since no sinks are
     * present, initialization always succeeds.
     *
     * @return ErrorCode::NoError.
     */
    ErrorCode init() override {
        return ErrorCode::NoError;
    };

    /**
     * @brief Writes binary log data.
     *
     * Base case implementation. No action is performed.
     *
     * @param data Pointer to the data buffer.
     * @param size Number of bytes to write.
     */
    void write(const uint8_t* data, size_t size) override {};

    /**
     * @brief Writes metadata.
     *
     * Base case implementation. No action is performed.
     *
     * @param data Pointer to the metadata buffer.
     * @param size Number of bytes to write.
     */
    void write_meta(const uint8_t* data, size_t size) override {};
};

template<typename Sink, typename... Sinks>
class MultipleLogSink<Sink, Sinks...> : public LogSink {
public:
    MultipleLogSink() = default;

    /**
     * @brief Constructs a recursive collection of logging sinks.
     *
     * @param sink_ First logging sink.
     * @param sinks_ Remaining logging sinks.
     */
    explicit MultipleLogSink(Sink sink_, Sinks... sinks_) : sink(sink_), sinks(sinks_...) { };

    /**
     * @brief Initializes every logging sink.
     *
     * Initialization proceeds recursively. If any sink fails to initialize,
     * initialization stops immediately and the corresponding error is returned.
     *
     * @return ErrorCode indicating success or the first initialization failure.
     */
    ErrorCode init() override {
        ErrorCode result = sink.init();
        if (result != ErrorCode::NoError) {
            return result;
        }
        return sinks.init();
    };

    /**
     * @brief Writes binary data to every logging sink.
     *
     * The write operation is forwarded recursively so each configured backend
     * receives an identical copy of the flight log.
     *
     * @param data Pointer to the data buffer.
     * @param size Number of bytes to write.
     */
    void write(const uint8_t* data, size_t size) override {
        sink.write(data, size);
        sinks.write(data, size);
    };

    /**
     * @brief Writes metadata to every logging sink.
     *
     * The metadata packet is forwarded recursively to each configured backend.
     *
     * @param data Pointer to the metadata buffer.
     * @param size Number of bytes to write.
     */
    void write_meta(const uint8_t* data, size_t size) override {
        sink.write(data, size);
        sinks.write(data, size);
    };

private:
    Sink sink;                          ///< Current logging backend.
    MultipleLogSink<Sinks...> sinks;    ///< Remaining logging backends.
};

#ifndef SILSIM
#include <FS.h>
/**
 * @brief Determines the next available log filename.
 *
 * Generates a unique filename using the provided base name and extension,
 * avoiding collisions with existing files on the filesystem.
 *
 * @param fileName Base filename buffer.
 * @param fileExtensionParam Desired file extension.
 * @param fs Filesystem to search.
 * @param file_num Starting file number.
 * @param fileno_out Pointer that receives the selected file number.
 *
 * @return Pointer to the generated filename.
 */
char* sdFileNamer(char* fileName, char* fileExtensionParam, FS& fs, uint16_t file_num, int* fileno_out);
#endif