#include <string.h>
#include <Queue.h>

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

struct MetaLogging {
    public:
        struct MetaLogEntry {
            MetaDataCode log_type;
            size_t size;
            char data[META_LOGGING_MAX_SIZE];
        };

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