#pragma once

#include <FS.h>
#include "sensors.h"
#include "errors.h"
#include "rocket_state.h"
#include "../data_logging.h"

class EmmcLog : public Logger {
    public:
        EmmcLog() = default;

        ErrorCode init();
        void write(const uint8_t* data, size_t size);
};
