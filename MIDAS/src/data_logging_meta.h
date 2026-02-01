#pragma once
#include "rocket_state.h"

#define META_LOGGING_MAX_SIZE 64

enum MetaDataCode {
    // Launch events
    EVENT_TLAUNCH,
    EVENT_TBURNOUT,
    EVENT_TIGNITION,
    EVENT_TAPOGEE,
    EVENT_TMAIN,

    // Non-events
    DATA_LAUNCHSITE_ALT_BARO,
    DATA_LAUNCHSITE_ALT_GPS,
    DATA_LAUNCH_INITIAL_TILT
};

struct MetaLogging {

    public:
    struct MetaLogEntry {
        MetaDataCode log_type;
        char data[META_LOGGING_MAX_SIZE];
        size_t size;
    };

    Queue<MetaLogEntry> _q;

    bool get_queued(MetaLogEntry* out) { return _q.receive(out); }

    void log_event(MetaDataCode event_type, uint32_t timestamp) {
        MetaLogEntry entry{event_type, 0, 0};
        entry.size = sizeof(uint32_t);
        memcpy(entry.data, &timestamp, entry.size);
        _q.send(entry);
    }

    template <typename T>
    void log_data(MetaDataCode data_type, const T& data) {

        // double check...
        static_assert(sizeof(T) <= META_LOGGING_MAX_SIZE, "Datatype for log_data too large");

        MetaLogEntry entry{data_type, 0, 0};
        entry.size = sizeof(T);
        memcpy(entry.data, &data, entry.size);
        _q.send(entry);
    }

};
