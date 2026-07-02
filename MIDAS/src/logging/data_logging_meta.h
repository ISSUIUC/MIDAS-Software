#include <string.h>
#include "util/Queue.h"
#include <algorithm>
#include <limits>
#include <flight-systems/sensor_data.h>

/// Maximum number of bytes that may be stored in a single metadata log entry.
#define META_LOGGING_MAX_SIZE 64

/**
 * @enum MetaDataCode
 *
 * @brief Identifiers for every metadata value that can be recorded during
 * flight.
 *
 * These identifiers are stored alongside metadata values so they can be
 * interpreted correctly during post-flight analysis.
 */
enum MetaDataCode {
    // Launch events
    EVENT_TLAUNCH,
    EVENT_TBURNOUT,
    EVENT_TIGNITION,
    EVENT_TAPOGEE,
    EVENT_TMAIN,
    EVENT_TMAX_ACCEL,
    EVENT_TMAX_VEL,
    EVENT_TMAX_DESCENT_RATE,

    // Non-events
    DATA_LAUNCHSITE_BARO,
    DATA_LAUNCHSITE_ALT,
    DATA_LAUNCHSITE_GPS_ALT,
    DATA_LAUNCHSITE_GPS_LAT,
    DATA_LAUNCHSITE_GPS_LONG,
    DATA_LAUNCH_INITIAL_TILT,
    DATA_TILT_AT_BURNOUT,
    DATA_TILT_AT_IGNITION,
    DATA_BARO_AT_IGNITION,
    DATA_MAX_ACCEL,
    DATA_MAX_VEL,
    DATA_ALT_AT_BURNOUT,
    DATA_MAX_DESCENT_RATE
};

/**
 * @enum MetalogSummaryEntryType
 *
 * @brief Specifies how a summary value should be updated throughout
 * flight.
 */
enum class MetalogSummaryEntryType {
    /// Always store the most recent value.
    CURRENT,

    /// Store only the maximum value observed.
    MAXIMUM,

    /// Store only the minimum value observed.
    MINIMUM,
};

struct MetalogSummary;

/**
 * @struct MetaLogging
 *
 * @brief Queues metadata entries for persistent storage.
 *
 * Metadata consists of significant flight events and summary values that
 * are written separately from the primary telemetry log.
 */
struct MetaLogging {
public:

    /**
     * @struct MetaLogEntry
     *
     * @brief Represents a single queued metadata record.
     */
    struct MetaLogEntry {

        /// Identifier describing the stored data.
        MetaDataCode log_type;

        /// Size of the stored payload in bytes.
        size_t size;

        /// Raw payload bytes.
        char data[META_LOGGING_MAX_SIZE];
    };

    /// Pointer to the metadata summary manager.
    MetalogSummary* summary;

    /// Queue of pending metadata records.
    Queue<MetaLogEntry> _q;

    /**
     * @brief Retrieves the next queued metadata entry.
     *
     * @param out Destination for the dequeued entry.
     *
     * @return true if an entry was available.
     * @return false if the queue was empty.
     */
    bool get_queued(MetaLogEntry* out) {
        return _q.receive(out);
    }

    /**
     * @brief Queues a metadata value for logging.
     *
     * The supplied object is copied directly into the metadata entry and
     * later written to persistent storage.
     *
     * @tparam T Type of the value being logged.
     *
     * @param data_type Metadata identifier.
     * @param data Value to record.
     */
    template <typename T>
    void log_data(MetaDataCode data_type, const T& data) {

        // Ensure the value fits within the fixed metadata payload buffer.
        static_assert(sizeof(T) <= META_LOGGING_MAX_SIZE,
                      "Datatype for log_data too large");

        // Create a metadata entry.
        MetaLogEntry entry{data_type, 0, 0};
        entry.size = sizeof(T);

        // Copy the raw bytes into the entry payload.
        memcpy(entry.data, &data, entry.size);

        // Queue the entry for later storage.
        _q.send(entry);
    }
};

/**
 * @class MetalogSummaryEntry
 *
 * @brief Tracks a single metadata summary statistic.
 *
 * A summary entry may store the current, maximum, or minimum value of a
 * quantity throughout the flight.
 *
 * @tparam T Type of value being tracked.
 */
template<typename T>
class MetalogSummaryEntry {

public:

    /**
     * @brief Constructs a metadata summary entry.
     *
     * @param metacode Metadata identifier.
     * @param metatype Update policy.
     * @param default_val Initial value.
     */
    MetalogSummaryEntry(const MetaDataCode& metacode,
                        const MetalogSummaryEntryType& metatype = MetalogSummaryEntryType::CURRENT,
                        const T& default_val = T()) {
        code = metacode;
        type = metatype;
        data = default_val;
    }

    /**
     * @brief Updates the tracked value.
     *
     * The update behavior depends on the configured summary type.
     *
     * @param newval New measurement.
     */
    void update(const T& newval) {
        switch(type) {
            case MetalogSummaryEntryType::CURRENT:
                data = newval;
                break;

            case MetalogSummaryEntryType::MAXIMUM:
                if (newval > data) {
                    data = newval;
                }
                break;

            case MetalogSummaryEntryType::MINIMUM:
                if (newval < data) {
                    data = newval;
                }
                break;
        }
    }

    /**
     * @brief Queues the tracked value for logging.
     *
     * @param metalog Metadata logger.
     */
    void commit(MetaLogging& metalog);

private:
    /// Stored summary value.
    T data;

    /// Metadata identifier.
    MetaDataCode code;

    /// Update policy.
    MetalogSummaryEntryType type;
};

/**
 * @struct MetalogSummary
 *
 * @brief Collection of all flight metadata summary entries.
 *
 * Stores timestamps for significant flight events and summary statistics
 * such as maximum acceleration, velocity, and descent rate.
 */
struct MetalogSummary {

    // Launch events
    MetalogSummaryEntry<uint32_t> event_tlaunch{MetaDataCode::EVENT_TLAUNCH};
    MetalogSummaryEntry<uint32_t> event_tburnout{MetaDataCode::EVENT_TBURNOUT};
    MetalogSummaryEntry<uint32_t> event_tignition{MetaDataCode::EVENT_TIGNITION};
    MetalogSummaryEntry<uint32_t> event_tapogee{MetaDataCode::EVENT_TAPOGEE};
    MetalogSummaryEntry<uint32_t> event_tmain{MetaDataCode::EVENT_TMAIN};
    MetalogSummaryEntry<uint32_t> event_tmax_accel{MetaDataCode::EVENT_TMAX_ACCEL};
    MetalogSummaryEntry<uint32_t> event_tmax_vel{MetaDataCode::EVENT_TMAX_VEL};
    MetalogSummaryEntry<uint32_t> event_tmax_descent_rate{MetaDataCode::EVENT_TMAX_DESCENT_RATE};

    // Flight summary values
    MetalogSummaryEntry<float> data_launchsite_baro{MetaDataCode::DATA_LAUNCHSITE_BARO};
    MetalogSummaryEntry<uint32_t> data_launchsite_gps_alt{MetaDataCode::DATA_LAUNCHSITE_GPS_ALT};
    MetalogSummaryEntry<uint32_t> data_launchsite_gps_lat{MetaDataCode::DATA_LAUNCHSITE_GPS_LAT};
    MetalogSummaryEntry<uint32_t> data_launchsite_gps_long{MetaDataCode::DATA_LAUNCHSITE_GPS_LONG};
    MetalogSummaryEntry<float> data_launch_initial_tilt{MetaDataCode::DATA_LAUNCH_INITIAL_TILT};
    MetalogSummaryEntry<float> data_tilt_at_burnout{MetaDataCode::DATA_TILT_AT_BURNOUT};
    MetalogSummaryEntry<float> data_tilt_at_ignition{MetaDataCode::DATA_TILT_AT_IGNITION};
    MetalogSummaryEntry<float> data_baro_at_ignition{MetaDataCode::DATA_BARO_AT_IGNITION};
    MetalogSummaryEntry<float> data_max_accel{MetaDataCode::DATA_MAX_ACCEL,
                                              MetalogSummaryEntryType::MAXIMUM,
                                              -std::numeric_limits<float>::max()};
    MetalogSummaryEntry<float> data_max_vel{MetaDataCode::DATA_MAX_VEL,
                                            MetalogSummaryEntryType::MAXIMUM,
                                            -std::numeric_limits<float>::max()};
    MetalogSummaryEntry<float> data_alt_at_burnout{MetaDataCode::DATA_ALT_AT_BURNOUT};
    MetalogSummaryEntry<float> data_max_descent_rate{MetaDataCode::DATA_MAX_DESCENT_RATE,
                                                     MetalogSummaryEntryType::MAXIMUM,
                                                     -std::numeric_limits<float>::max()};
};

/**
 * @brief Commits the stored summary value to the metadata logger.
 *
 * @param metalog Metadata logger.
 */
template <typename T>
void MetalogSummaryEntry<T>::commit(MetaLogging& metalog) {
    metalog.log_data(code, data);
}