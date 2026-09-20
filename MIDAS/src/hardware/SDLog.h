#pragma once

#include <FS.h>

#include <SD.h>
#include "logging/data_logging.h"

/**
 * @class SDSink
 *
 * @brief Log sink that writes flight data to an SD card.
 *
 * Manages separate files for telemetry data and metadata while buffering
 * writes to reduce the number of SD card flush operations.
 */
class SDSink : public LogSink {
public:
    /**
     * @brief Indicates whether SD card initialization or a write operation
     *        has failed.
     */
    bool failed = false;

    /**
     * @brief Constructs an SD log sink.
     */
    SDSink() = default;

    /**
     * @brief Initializes the SD card and opens the log files.
     *
     * @return Error code indicating whether initialization succeeded.
     */
    ErrorCode init() override;

    /**
     * @brief Writes flight log data to the SD card.
     *
     * @param data Pointer to the data buffer.
     * @param size Number of bytes to write.
     */
    void write(const uint8_t* data, size_t size) override;

    /**
     * @brief Writes metadata to the SD card.
     *
     * Metadata is stored separately from the primary flight log.
     *
     * @param data Pointer to the metadata buffer.
     * @param size Number of bytes to write.
     */
    void write_meta(const uint8_t* data, size_t size) override;

private:
    /// Flight data log file.
    File file;

    /// Metadata log file.
    File meta;

    /**
     * @brief Number of bytes written since the last file flush.
     *
     * Used to reduce unnecessary SD card synchronization operations.
     */
    size_t unflushed_bytes = 0;
};