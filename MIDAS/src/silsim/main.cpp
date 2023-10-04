#include "silsim/emulation.h"
#include "systems.h"

int main() {
    begin_silsim();

    RocketSystems systems;
    begin_systems(systems);
    return 0;
}