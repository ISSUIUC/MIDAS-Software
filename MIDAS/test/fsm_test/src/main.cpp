/**
 * FSM Simulation (C++ variant)
 * @author Michael Karpov (2026)
 * @brief A C++, SIL-based version of the FSM tester.
 */

#include <cstdio>
#include <cstring>
#include <string>

#include "csv.hpp"
#include "config_parser.h"
#include "finite-state-machines/fsm.h"
#include "finite-state-machines/pyro_eval.h"
#include "flight-systems/sensor_data.h"
#include "finite-state-machines/command_flags.h"

static void print_usage(const char* prog) {
    fprintf(stderr,
        "Usage: %s <input.csv> <output.csv>\n"
        "\n"
        "Input CSV format:\n"
        "  The file begins with optional #cfg: header lines, followed by data rows.\n"
        "  Lines not starting with '#' are parsed as CSV with these columns:\n"
        "    timestamp_ms, acceleration_g, altitude_m, vertical_speed_mps, vx_mps, tilt_deg\n"
        "    (vx_mps and tilt_deg are optional, default to 0)\n"
        "\n"
        "Configuration header (#cfg: lines):\n"
        "  #cfg:initial_state=<0-8>          Initial FSM state (default: 2 = ARMED)\n"
        "                                      0=SAFE 1=PYRO_TEST 2=ARMED 3=BOOST\n"
        "                                      4=COAST 5=APOGEE 6=DROGUE 7=MAIN 8=LANDED\n"
        "  #cfg:main_alt=<meters>            Main parachute deployment altitude\n"
        "  #cfg:cruise_lockout_en=<bool>     Enable cruise velocity lockout (true/false)\n"
        "  #cfg:pyro_fire_t=<ms>             Duration to keep pyro channel firing\n"
        "  #cfg:pyro_<0-3>=<fields>          Pyro channel config (comma-separated key=value):\n"
        "      enable=<bool>                   Enable this channel\n"
        "      fsm_trigger=<0-8>               FSM state that triggers firing\n"
        "      max_tilt=<deg>                  Max tilt angle for firing (-1 = disabled)\n"
        "      after_motor=<n>                 Fire after motor n (0 = any)\n"
        "      launch_t_gt=<sec>               Min time since launch (-1 = disabled)\n"
        "      launch_t_lt=<sec>               Max time since launch (-1 = disabled)\n"
        "      vx_min=<m/s>                    Min velocity (-1 = disabled)\n"
        "      vx_max=<m/s>                    Max velocity (-1 = disabled)\n"
        "      delay=<sec>                     Delay after trigger before firing\n"
        "\n"
        "Example:\n"
        "  #cfg:initial_state=2\n"
        "  #cfg:main_alt=800.0\n"
        "  #cfg:pyro_fire_t=10.0\n"
        "  #cfg:pyro_0=enable=true,fsm_trigger=6,max_tilt=-1,after_motor=0,launch_t_gt=-1,launch_t_lt=-1,vx_min=-1,vx_max=-1,delay=0\n"
        "  timestamp_ms,acceleration_g,altitude_m,vertical_speed_mps\n"
        "  0.000,1.000,100.000,0.0000\n"
        "  ...\n",
        prog);
}

int main(int argc, char** argv) {
    if (argc < 3 || (argc >= 2 && (strcmp(argv[1], "--help") == 0 || strcmp(argv[1], "-h") == 0))) {
        print_usage(argv[0]);
        return (argc < 3) ? 1 : 0;
    }

    const char* input_path = argv[1];
    const char* output_path = argv[2];

    //parse the config first
    FILE* cfg_fp = fopen(input_path, "r");
    if (!cfg_fp) {
        fprintf(stderr, "Cannot open csv: %s\n", input_path);
        return 1;
    }
    SimConfig sim = parse_config_lines(cfg_fp);
    fclose(cfg_fp);

    //set up fsm configuration
    FSM fsm;
    if (!fsm.set_cfg(sim.fsm_config)) {
        fprintf(stderr, "FSM configuration CRC mismatch\n");
        return 1;
    }

    FSMData state_data = {};
    state_data.state = sim.initial_state;
    state_data.current_motor = 0;

    CommandFlags commands = {};
    KalmanData kf_data = {};
    PyroEvalState pyro_state = {};
    FSMState prev_state = sim.initial_state;

    // write to csv
    FILE* out_fp = fopen(output_path, "w");
    if (!out_fp) {
        fprintf(stderr, "Cannot open output: %s\n", output_path);
        return 1;
    }

    fprintf(out_fp, "timestamp_ms,state,acceleration_g,altitude_m,vertical_speed_mps,vx_mps,tilt_deg,cruise_lockout");
    for (int i = 0; i < MIDAS_NUM_PYROS; i++) {
        fprintf(out_fp, ",pyro_%d_triggered,pyro_%d_firing,pyro_%d_consumed", i, i, i);
    }
    fprintf(out_fp, "\n");

    // read the actual data
    csv::CSVReader reader(input_path);

    for (csv::CSVRow& row : reader) {
        double timestamp_ms = row["timestamp_ms"].get<double>();
        double acceleration_g = row["acceleration_g"].get<double>();
        double altitude_m = row["altitude_m"].get<double>();
        double vertical_speed_mps = row["vertical_speed_mps"].get<double>();
        double vx_mps = 0.0;
        try { vx_mps = row["vx_mps"].get<double>(); } catch (...) {}
        double tilt_deg = 0.0;
        try { tilt_deg = row["tilt_deg"].get<double>(); } catch (...) {}

        StateEstimate estimate(altitude_m, acceleration_g, 0.0, vertical_speed_mps);
        kf_data.velocity.vx = (float)vx_mps;

        FSMTickData tick_data = {
            .cur_state = state_data,
            .commands = commands,
            .state_estimate = estimate,
            .kf_data = kf_data,
            .fsm_config = sim.fsm_config,
            .cur_time = timestamp_ms
        };

        fsm.tick_fsm(tick_data);

        float time_since_launch = (float)(timestamp_ms - fsm.get_launch_time());
        PyroEvalResult pyro_result = {};

        if (sim.fsm_config.crc32 != FSM_CRC_FAIL_STATE &&
            state_data.state != STATE_SAFE &&
            state_data.state != STATE_ARMED &&
            state_data.state != STATE_PYRO_TEST) {

            pyro_result = pyro_eval(
                sim.fsm_config, state_data, tilt_deg,
                time_since_launch, kf_data.velocity.vx,
                timestamp_ms, pyro_state
            );
        }

        // cruise lockout: enabled in config AND vx exceeds threshold
        bool cruise_lockout = sim.fsm_config.thresholds.cruise_lockout_en &&
                              (kf_data.velocity.vx > fsms_cruise_lockout_spd);

        // write line
        fprintf(out_fp, "%.3f,%d,%.6f,%.3f,%.4f,%.4f,%.4f,%d",
                timestamp_ms, (int)state_data.state,
                acceleration_g, altitude_m, vertical_speed_mps, vx_mps, tilt_deg,
                cruise_lockout ? 1 : 0);
        for (int i = 0; i < MIDAS_NUM_PYROS; i++) {
            fprintf(out_fp, ",%d,%d,%d",
                    pyro_result.event_triggered[i] ? 1 : 0,
                    pyro_result.channel_firing[i] ? 1 : 0,
                    pyro_result.event_consumed[i] ? 1 : 0);
        }
        fprintf(out_fp, "\n");
    }

    fclose(out_fp);
    printf("Output written to %s\n", output_path);
    return 0;
}
