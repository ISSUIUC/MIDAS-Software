#pragma once

#include <FS.h>
#include "sensors.h"
#include "errors.h"
#include "rocket_state.h"

struct EmmcLog : public Logger {
    ErrorCode init();
    void write(const uint8_t* data, size_t size);
};
