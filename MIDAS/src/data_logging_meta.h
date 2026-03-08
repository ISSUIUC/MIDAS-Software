#pragma once
//#include "rocket_state.h"
//#include "errors.h"

/*

#define META_LOGGING_MAX_SIZE 64

enum MetaDataCode {
    // Launch events
    EVENT_TLAUNCH,
    EVENT_TBURNOUT,
    EVENT_TIGNITION,
    EVENT_TAPOGEE,
    EVENT_TMAIN,

    // Non-events
    DATA_LAUNCHSITE_BARO,
    DATA_LAUNCHSITE_GPS,
    DATA_LAUNCH_INITIAL_TILT,
    DATA_TILT_AT_BURNOUT,
    DATA_TILT_AT_IGNITION
};

struct MetaLogging {

    public:
    struct MetaLogEntry {
        MetaDataCode log_type;
        char data[META_LOGGING_MAX_SIZE];
        size_t size;
    };

    bool get_queued(MetaLogEntry* out);

    void log_event(MetaDataCode event_type, uint32_t timestamp);

    template <typename T>
    void log_data(MetaDataCode data_type, const T& data);
};

*/
