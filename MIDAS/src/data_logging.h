#pragma once

#include "hardware_interface.h"
#include "rocket_state.h"

void log_begin(ILogSink& sink);
void log_data(ILogSink& sink, RocketData& data);
