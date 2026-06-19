#pragma once

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include "fsm.h"

struct SimConfig {
    FSMState initial_state = STATE_ARMED;
    FSMConfiguration fsm_config = {};
};

inline FSMState state_from_int(const std::string& s) {
    int v = atoi(s.c_str());
    if (v >= 0 && v < FSM_STATE_COUNT) return (FSMState)v;
    return STATE_SAFE;
}

// helper for pyro= lines
inline FSMPyroAction parse_pyro_line(const std::string& val) {
    FSMPyroAction a = {};

    std::string buf = val;
    char* saveptr = nullptr;
    char* tok = strtok_r(&buf[0], ",", &saveptr);
    while (tok) {
        char* eq = strchr(tok, '=');
        if (eq) {
            std::string key(tok, eq - tok);
            std::string v(eq + 1);
            if (key == "enable") a.enable = (v == "true" || v == "1");
            else if (key == "fsm_trigger") a.fsm_trigger = state_from_int(v);
            else if (key == "max_tilt") a.max_tilt = strtof(v.c_str(), nullptr);
            else if (key == "after_motor") a.after_motor = (uint8_t)atoi(v.c_str());
            else if (key == "launch_t_gt") a.launch_t_gt = strtof(v.c_str(), nullptr);
            else if (key == "launch_t_lt") a.launch_t_lt = strtof(v.c_str(), nullptr);
            else if (key == "vx_min") a.vx_min = strtof(v.c_str(), nullptr);
            else if (key == "vx_max") a.vx_max = strtof(v.c_str(), nullptr);
            else if (key == "delay") a.delay = strtod(v.c_str(), nullptr);
        }
        tok = strtok_r(nullptr, ",", &saveptr);
    }
    return a;
}

// parse #cfg
inline SimConfig parse_config_lines(FILE* fp) {
    SimConfig cfg;
    char line[4096];
    long pos = 0;

    while (fgets(line, sizeof(line), fp)) {
        if (line[0] != '#') {
            fseek(fp, pos, SEEK_SET);
            break;
        }
        pos = ftell(fp);

        // check for `#cfg:`
        const char* prefix = "#cfg:";
        if (strncmp(line, prefix, strlen(prefix)) != 0) continue;

        // get rid of newline
        char* nl = strchr(line, '\n');
        if (nl) *nl = '\0';
        nl = strchr(line, '\r');
        if (nl) *nl = '\0';

        std::string content(line + strlen(prefix));
        auto eq = content.find('=');
        if (eq == std::string::npos) continue;

        std::string key = content.substr(0, eq);
        std::string val = content.substr(eq + 1);

        if (key == "initial_state") {
            cfg.initial_state = state_from_int(val);
        } else if (key == "main_alt") {
            cfg.fsm_config.thresholds.main_alt = strtof(val.c_str(), nullptr);
        } else if (key == "cruise_lockout_en") {
            cfg.fsm_config.thresholds.cruise_lockout_en = (val == "true" || val == "1");
        } else if (key == "pyro_fire_t") {
            cfg.fsm_config.thresholds.pyro_fire_t = strtof(val.c_str(), nullptr);
        } else if (key.size() == 6 && key.substr(0, 5) == "pyro_") {
            int idx = key[5] - '0';
            if (idx >= 0 && idx < MIDAS_NUM_PYROS) {
                cfg.fsm_config.pyro_actions[idx] = parse_pyro_line(val);
            }
        }
    }

    cfg.fsm_config.version_num = 1;
    cfg.fsm_config.crc32 = FSMConfiguration::calculate_crc(cfg.fsm_config);
    return cfg;
}
