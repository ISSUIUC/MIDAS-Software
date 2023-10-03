# MIDAS Flight Code

## Getting Started

Clone this whole repository onto your computer somewhere.

// todo explain how to install/use platformio
// todo have both VSCode and CLion sections

### Command Line
If you want to run SILSIM, you'll need `gcc` to be available on your path. 
On Linux/MacOS, `gcc` should be preinstalled. 
On Windows, you'll need to install the newest version of MinGW-w64 and add it's
`\bin` directory to your PATH environment variable.

To flash flight code to the MIDAS board using the command line, run
`pio run -e mcu_main -t upload`. To just build flight code for MIDAS without
actually uploading it (to make sure it compiles), use `pio run -e mcu_main`.
Note that if you never directly installed platformio and instead are just using
the VSCode extension, these commands won't work until you install platformio
manually.

To run SILSIM from the command line, use `pio run -e mcu_silsim`.

## Design Decisions

In order to support staging in SILSIM, the flight code has been designed
with a "no globals" architecture, so that multiple instances of flight code
can be running at the same time. The primary tool we use to do this is the
`RocketSystems` struct in `systems.h`. The entry point to the real flight
code is hidden in `systems.cpp`, and it takes a `RocketSystems` parameter
which provides the actual sensors and data-logging functionality needed by
the systems in flight code.

In order to maximise the amount of code being re-used between SILSIM, HILSIM,
and the actual flight code, anything which has to be different between the 
three modes must be inside one of the three folders `hardware`, `hilsim`, and 
`silsim`. By switching out which of the three folders we use during compilation,
we can switch which one we are using. Additionally, the `sensors.h` file
provides a "protocol" of sorts for the sensors used in the struct `Sensors`,
which abstracts over the differences between the emulated sensors of HILSIM
and SILIM, and the hardware sensors of true flight code.