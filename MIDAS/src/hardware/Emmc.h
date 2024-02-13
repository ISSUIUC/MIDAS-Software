#pragma once

#include <FS.h>
#include <SD_MMC.h>

#include "data_logging.h"
#include "sensors.h"
#include "errors.h"
#include "rocket_state.h"

class EMMCSink : public LogSink {
public:
    EMMCSink() = default;

    ErrorCode init();
    void write(const uint8_t* data, size_t size);
private:
    File file;
};
