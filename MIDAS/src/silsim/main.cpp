#include "silsim/emulation.h"
#include "systems.h"

RocketSystems systems;

void rocket_main(void*){
    begin_systems(systems);
}

int main() {
    begin_silsim();

    uint8_t stack[4096];
    xTaskCreateStaticPinnedToCore(rocket_main, "rocket main", 4096, nullptr, tskIDLE_PRIORITY, stack, nullptr, 0);

    for(int i = 0; i < 1000 * 100; i++){
        // phsyics
        
        silsimStepTime();
    }

    return 0;
}