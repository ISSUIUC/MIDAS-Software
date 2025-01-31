#include "emulated_telemetry.h"
#include "silsim/emulated_sensors.h"
#include "silsim/emulation.h"
#include "systems.h"
#include "silsim/FileSink.h"
#include "simulation/simulation.h"

void rocket_main_thread(RocketSystems* systems) {
    systems->begin();
}

RocketSystems* create_and_start(RocketState& state, const char* sd_sink_name, const char* tlm_sink_name) {
    SDSink* sink = new SDSink(sd_sink_name);
    RocketSystems* systems = new RocketSystems(Sensors {
            .low_g = *new LowGSensor(state),
            .low_g_lsm = *new LowGLSMSensor(state),
            .high_g = *new HighGSensor(state),
            .barometer = *new BarometerSensor(state),
            .continuity = *new ContinuitySensor(state),
            .voltage = *new VoltageSensor(state),
            .orientation = *new OrientationSensor(state),
            .magnetometer = *new MagnetometerSensor(state),
            .gps = *new GPSSensor(state),
            .sink = *sink,
            .telemetry = *new TelemetryBackend(tlm_sink_name),
            .buzzer = *new BuzzerBackend(),
            .led = *new LedBackend(),
            .pyro = *new PyroBackend()
    });

    xTaskCreateStaticPinnedToCore((TaskFunction_t) rocket_main_thread, sd_sink_name, 4096, systems, tskIDLE_PRIORITY, new uint8_t[4096], nullptr, 0);
    return systems;
}

constexpr uint32_t DT_MS = 1;

int main() {
    begin_silsim();

    RocketState stage1;
    RocketState stage2;

    Simulation sim { .density_of_air = 0.001, .gravity = 9.81 };

    RocketSystems* stage1_systems = create_and_start(stage1, "stage1_sink.launch", "stage1_tlm_sink.bin");
    /* RocketSystems* stage2_systems = */ create_and_start(stage2, "stage2_sink.launch", "stage2_tlm_sink.bin");

    RocketParameters together_rocket {
        .mass = 1,
        .cross_sectional_area = 1,
        .drag_coefficient = 0.75,
        .ignition_time = 0,
        .motor = SimulatedMotor(100.0, 10.0)
    };
    while (!stage1_systems->rocket_data.pyro.getRecent().channels[0].is_firing && (stage1.height != 0 || silsim_current_time() < 0.1)) {
        sim.step_rocket(stage1, together_rocket, DT_MS / 1000.0);
        sim.step_rocket(stage2, together_rocket, DT_MS / 1000.0);

        silsim_step_time(DT_MS, 1);

        std::cout << "Rocket altitude: " << stage1.height << " velocity: " << stage1.velocity << std::endl;
    }

    RocketParameters stage1_rocket {
        .mass = 0.5,
        .cross_sectional_area = 1,
        .drag_coefficient = 0.75,
        .ignition_time = silsim_current_time(),
        .motor = SimulatedMotor(100.0, 10.0)
    };
    RocketParameters stage2_rocket {
        .mass = 0.5,
        .cross_sectional_area = 1,
        .drag_coefficient = 0.75,
        .ignition_time = silsim_current_time(),
        .motor = SimulatedMotor(0.0, 10.0)
    };
    while (stage1.height != 0 && stage2.height != 0) {
        sim.step_rocket(stage1, stage1_rocket, DT_MS / 1000.0);
        sim.step_rocket(stage2, stage2_rocket, DT_MS / 1000.0);

        silsim_step_time(DT_MS, 1);
    }
}
