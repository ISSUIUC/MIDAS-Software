#include "silsim/emulation.h"
#include "systems.h"

void rocket_main(RocketSystems* systems) {
    begin_systems(systems);
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
        Sensors {
            LowGSensor { together_sim },
            HighGSensor { together_sim },
            BarometerSensor { together_sim },
            ContinuitySensor { together_sim },
            VoltageSensor { together_sim },
            OrientationSensor { together_sim }
        },
    };
    stage1_systems->log_sink.output_file.open("stage1_sink.launch", std::ios::out | std::ios::binary | std::ios::trunc);

    uint8_t stack[4096];
    xTaskCreateStaticPinnedToCore((TaskFunction_t) rocket_main, "rocket main", 4096, stage1_systems, tskIDLE_PRIORITY, stack, nullptr, 0);

    while (true) {
        sim.step(0.001);

        silsimStepTime();
    }

    return 0;
}