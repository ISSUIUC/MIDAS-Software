#pragma once

#include <hardware_interface.h>

class LedBackend final: public ILedBackend {
public:
    void set(LED led, bool on) override;
};
