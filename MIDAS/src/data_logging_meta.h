#include <string.h>
#include <Queue.h>
#include <algorithm>
#include <limits>

#define META_LOGGING_MAX_SIZE 64

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
    DATA_LAUNCHSITE_GPS,
    DATA_LAUNCH_INITIAL_TILT,
    DATA_TILT_AT_BURNOUT,
    DATA_TILT_AT_IGNITION,
    DATA_MAX_ACCEL,
    DATA_MAX_VEL,
    DATA_ALT_AT_BURNOUT,
    DATA_MAX_DESCENT_RATE
};

enum class MetalogSummaryEntryType{
    CURRENT,
    MAXIMUM,
    MINIMUM,
};

struct MetalogSummary;

struct MetaLogging {
    public:
        struct MetaLogEntry {
            MetaDataCode log_type;
            size_t size;
            char data[META_LOGGING_MAX_SIZE];
        };

        MetalogSummary * summary;

        Queue<MetaLogEntry> _q;

        bool get_queued(MetaLogEntry* out) { return _q.receive(out); }

        template <typename T>
        void log_data(MetaDataCode data_type, const T& data) {

            // double check...
            static_assert(sizeof(T) <= META_LOGGING_MAX_SIZE, "Datatype for log_data too large");

            MetaLogEntry entry{data_type, 0, 0};
            entry.size = sizeof(T);
            memcpy(entry.data, &data, entry.size);
            _q.send(entry);

            // fprintf(stderr, "Data has been logged: %c", entry.data);
        }
};

template<typename T>
class MetalogSummaryEntry {

    public:

    MetalogSummaryEntry(const MetaDataCode &metacode, const MetalogSummaryEntryType &metatype = MetalogSummaryEntryType::CURRENT, const T &default_val = T()){
        code = metacode;
        type = metatype;
        data = default_val;
    }

    void update(const T &newval){
        switch(type){
            case MetalogSummaryEntryType::CURRENT:
                data = newval;
                break;
            case MetalogSummaryEntryType::MAXIMUM:
                if (newval > data){
                    data = newval;
                }
                break;
            case MetalogSummaryEntryType::MINIMUM:
                if (newval < data){
                    data = newval;
                }
                break;
        }
    }

    void commit(MetaLogging &metalog);

    private:
        T data;
        MetaDataCode code;
        MetalogSummaryEntryType type;  
};

struct MetalogSummary{
    // Launch events
    MetalogSummaryEntry<uint32_t> event_tlaunch {MetaDataCode::EVENT_TLAUNCH};
    MetalogSummaryEntry<uint32_t> event_tburnout {MetaDataCode::EVENT_TBURNOUT};
    MetalogSummaryEntry<uint32_t> event_tignition {MetaDataCode::EVENT_TIGNITION};
    MetalogSummaryEntry<uint32_t> event_tapogee {MetaDataCode::EVENT_TAPOGEE};
    MetalogSummaryEntry<uint32_t> event_tmain {MetaDataCode::EVENT_TMAIN};
    MetalogSummaryEntry<uint32_t> event_tmax_accel {MetaDataCode::EVENT_TMAX_ACCEL};
    MetalogSummaryEntry<uint32_t> event_tmax_vel {MetaDataCode::EVENT_TMAX_VEL};
    MetalogSummaryEntry<uint32_t> event_tmax_descent_rate {MetaDataCode::EVENT_TMAX_DESCENT_RATE};

    // Non-events
    MetalogSummaryEntry<float> data_launchsite_baro {MetaDataCode::DATA_LAUNCHSITE_BARO};
    MetalogSummaryEntry<float> data_launchsite_gps {MetaDataCode::DATA_LAUNCHSITE_GPS};
    MetalogSummaryEntry<float> data_launch_initial_tilt {MetaDataCode::DATA_LAUNCH_INITIAL_TILT};
    MetalogSummaryEntry<float> data_tilt_at_burnout {MetaDataCode::DATA_TILT_AT_BURNOUT};
    MetalogSummaryEntry<float> data_tilt_at_ignition {MetaDataCode::DATA_TILT_AT_IGNITION};
    MetalogSummaryEntry<float> data_max_accel {MetaDataCode::DATA_MAX_ACCEL, MetalogSummaryEntryType::MAXIMUM, -std::numeric_limits<float>::max()};
    MetalogSummaryEntry<float> data_max_vel {MetaDataCode::DATA_MAX_VEL, MetalogSummaryEntryType::MAXIMUM, -std::numeric_limits<float>::max()};
    MetalogSummaryEntry<float> data_alt_at_burnout {MetaDataCode::DATA_ALT_AT_BURNOUT};
    MetalogSummaryEntry<float> data_max_descent_rate {MetaDataCode::DATA_MAX_DESCENT_RATE, MetalogSummaryEntryType::MAXIMUM, -std::numeric_limits<float>::max()};
};

template <typename T>
void MetalogSummaryEntry<T>::commit(MetaLogging &metalog){metalog.log_data(code, data);}

