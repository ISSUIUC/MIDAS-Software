#pragma once

#include <FS.h>

#include "sensors.h"

#include "errors.h"

struct EmmcLog {
    ErrorCode init();

    File file;
};