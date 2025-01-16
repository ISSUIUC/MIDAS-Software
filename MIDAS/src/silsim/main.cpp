#include "emulated_telemetry.h"
#include "silsim/emulated_sensors.h"
#include "silsim/emulation.h"
#include "systems.h"
#include "silsim/FileSink.h"
#include "simulation/simulation.h"

void rocket_main_thread(RocketSystems* systems) {
    systems->begin();
}

RocketSystems* create_and_start(SimulatedRocket** sim, const char* sd_sink_name, const char* tlm_sink_name) {
    // std::cerr << "Could not open file!\n";
    SDSink* sink = new SDSink(sd_sink_name);
    // std::cerr << "Could not open file!\n";
    RocketSystems* systems = new RocketSystems(Sensors {
            .low_g = *new LowGSensor(sim),
            .low_g_lsm = *new LowGLSMSensor(sim),
            .high_g = *new HighGSensor(sim),
            .barometer = *new BarometerSensor(sim),
            .continuity = *new ContinuitySensor(sim),
            .voltage = *new VoltageSensor(sim),
            .orientation = *new OrientationSensor(sim),
            .magnetometer = *new MagnetometerSensor(sim),
            .gps = *new GPSSensor(sim),
            .sink = *sink,
            .telemetry = *new TelemetryBackend(tlm_sink_name),
            .buzzer = *new BuzzerBackend(),
            .led = *new LedBackend(),
            .pyro = *new PyroBackend()
    });

    uint8_t* stack = new uint8_t[4096];
    xTaskCreateStaticPinnedToCore((TaskFunction_t) rocket_main_thread, sd_sink_name, 4096, systems, tskIDLE_PRIORITY, stack, nullptr, 0);
    return systems;
}

int main() {
    begin_silsim();

    SimulatedRocket* together_sim = new SimulatedRocket(true, RocketParameters(1, 1, 0.75, SimulatedMotor(100.0, 10.0)));
    SimulatedRocket* stage1_sim = together_sim;
    SimulatedRocket* stage2_sim = together_sim;

    Simulation sim({ .density_of_air = 0.001, .gravity = 9.81 }, std::vector { together_sim, stage1_sim, stage2_sim });

    RocketSystems* stage1_systems = create_and_start(&stage1_sim, "stage1_sink.launch", "stage1_tlm_sink.bin");
    RocketSystems* stage2_systems = create_and_start(&stage2_sim, "stage2_sink.launch", "stage2_tlm_sink.bin");

    sim.ignite_rocket(together_sim);

    while (!stage1_systems->rocket_data.pyro.getRecent().channels[0].is_firing && (together_sim->height != 0 || sim.current_time < 0.1)) {
        sim.step(0.001);

        silsimStepTime();

        std::cout << "Rocket altitude: " << together_sim->height << " velocity: " << together_sim->velocity << std::endl;
    }

    stage1_sim = new SimulatedRocket(false, RocketParameters(0.5, 1, 0.75, SimulatedMotor(100.0, 10.0)));
    stage2_sim = new SimulatedRocket(false, RocketParameters(0.5, 1, 0.75, SimulatedMotor(0.0, 10.0)));
    stage1_sim->copy_state_from(together_sim);
    stage2_sim->copy_state_from(together_sim);
    sim.activate_rocket(stage1_sim);
    sim.activate_rocket(stage2_sim);
    sim.deactivate_rocket(together_sim);
    sim.ignite_rocket(stage2_sim);

    while (stage2_sim->height != 0 && stage1_sim->height != 0) {
        sim.step(0.001);

        silsimStepTime();
    }
}
