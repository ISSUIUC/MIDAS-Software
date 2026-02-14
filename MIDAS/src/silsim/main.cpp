//#include "silsim/emulation.h"
#include "systems.h"
#include "FileSink.h"

void rocket_main_thread(RocketSystems* systems) {
    begin_systems(systems);
}

Sensors create_sensors_attached_to(SimulatedRocket* sim, bool should_be_continuous) {
    return Sensors {
            LowGSensor { sim },
            LowGLSMSensor { sim },
            HighGSensor { sim },
            BarometerSensor { sim },
            VoltageSensor { sim },
            OrientationSensor { sim },
            MagnetometerSensor { sim },
    };
}

RocketSystems* create_and_start(SimulatedRocket* attached, const char* sink_name) {
    SDSink* sink = new SDSink(sink_name);
    RocketSystems* systems = new RocketSystems {
            create_sensors_attached_to(attached, true),
            {},
            *sink
    };

    uint8_t* stack = new uint8_t[4096];
    xTaskCreateStaticPinnedToCore((TaskFunction_t) rocket_main_thread, sink_name, 4096, systems, tskIDLE_PRIORITY, stack, nullptr, 0);
    return systems;
}

int main() {
    begin_silsim();

    SimulatedRocket* together_sim = new SimulatedRocket(true, RocketParameters(1, 1, 0.75, SimulatedMotor(100.0, 10.0)));
    SimulatedRocket* stage1_sim = new SimulatedRocket(false, RocketParameters(0.5, 1, 0.75, SimulatedMotor(100.0, 10.0)));
    SimulatedRocket* stage2_sim = new SimulatedRocket(false, RocketParameters(0.5, 1, 0.75, SimulatedMotor(0.0, 10.0)));

    Simulation sim({ .density_of_air = 0.001, .gravity = 9.81 }, std::vector<SimulatedRocket*> { together_sim, stage1_sim, stage2_sim });

    RocketSystems* stage1_systems = create_and_start(together_sim, "stage1_sink.launch");
    RocketSystems* stage2_systems = create_and_start(together_sim, "stage2_sink.launch");

    sim.ignite_rocket(together_sim);

    while (!stage1_systems->rocket_data.pyro_should_be_firing && (together_sim->height != 0 || sim.current_time < 0.1)) {
        sim.step(0.001);

        silsimStepTime();

        std::cout << "Rocket altitude: " << together_sim->height << " velocity: " << together_sim->velocity << std::endl;
    }

    stage1_sim->copy_state_from(together_sim);
    stage2_sim->copy_state_from(together_sim);
    sim.activate_rocket(stage1_sim);
    sim.activate_rocket(stage2_sim);
    sim.deactivate_rocket(together_sim);
    sim.ignite_rocket(stage2_sim);
    stage1_systems->sensors = create_sensors_attached_to(stage1_sim, false);
    stage2_systems->sensors = create_sensors_attached_to(stage2_sim, false);

    while (stage2_sim->height != 0 && stage1_sim->height != 0) {
        sim.step(0.001);

        silsimStepTime();
    }
}
