#include "silsim/emulation.h"
#include "systems.h"

void rocket_main_thread(RocketSystems* systems) {
    begin_systems(systems);
}

void start_rocket_thread(const char* name, RocketSystems* systems) {
    uint8_t* stack = new uint8_t[4096];
    xTaskCreateStaticPinnedToCore((TaskFunction_t) rocket_main_thread, name, 4096, systems, tskIDLE_PRIORITY, stack, nullptr, 0);
}

Sensors create_sensors_attached_to(SimulatedRocket* sim) {
    return Sensors {
            LowGSensor { sim },
            HighGSensor { sim },
            BarometerSensor { sim },
            ContinuitySensor { sim },
            VoltageSensor { sim },
            OrientationSensor { sim }
    };
}

int main() {
    begin_silsim();

    SimulatedRocket* together_sim = new SimulatedRocket(true, RocketParameters(1, 1, 0.75, SimulatedMotor(10.0, 10.0)));
    SimulatedRocket* stage1_sim = new SimulatedRocket(false, RocketParameters(0.5, 1, 0.75, SimulatedMotor(10.0, 10.0)));
    SimulatedRocket* stage2_sim = new SimulatedRocket(false, RocketParameters(0.5, 1, 0.75, SimulatedMotor(0.0, 10.0)));

    Simulation sim {
        0.0,
        { together_sim, stage1_sim, stage2_sim },
    };

    RocketSystems* stage1_systems = new RocketSystems {
            create_sensors_attached_to(together_sim)
    };
    stage1_systems->log_sink.output_file.open("stage1_sink.launch", std::ios::out | std::ios::binary | std::ios::trunc);
    start_rocket_thread("stage1 thread", stage1_systems);

    RocketSystems* stage2_systems = new RocketSystems {
        create_sensors_attached_to(together_sim)
    };
    stage2_systems->log_sink.output_file.open("stage2_sink.launch", std::ios::out | std::ios::binary | std::ios::trunc);
    start_rocket_thread("stage2 thread", stage2_systems);

    sim.ignite_rocket(together_sim);

    while (!stage1_systems->rocket_data.pyro_should_be_firing) {
        sim.step(0.001);

        silsimStepTime();
    }

    stage1_sim->copy_state_from(together_sim);
    stage2_sim->copy_state_from(together_sim);
    sim.activate_rocket(stage1_sim);
    sim.activate_rocket(stage2_sim);
    sim.deactivate_rocket(together_sim);
    sim.ignite_rocket(stage1_sim);
    stage1_systems->sensors = create_sensors_attached_to(stage1_sim);
    stage2_systems->sensors = create_sensors_attached_to(stage2_sim);

    while (stage2_sim->height != 0) {
        sim.step(0.001);

        silsimStepTime();
    }
}