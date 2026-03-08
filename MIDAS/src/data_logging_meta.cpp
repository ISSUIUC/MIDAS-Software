#include "data_logging_meta.h"
#include "log_format.h"
#include "log_checksum.h"

/*

Queue<MetaLogging::MetaLogEntry> _q;

bool MetaLogging::get_queued(MetaLogEntry* out) { return _q.receive(out); }

void log_event(MetaDataCode event_type, uint32_t timestamp) {
    MetaLogging::MetaLogEntry entry{event_type, 0, 0};
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

*/