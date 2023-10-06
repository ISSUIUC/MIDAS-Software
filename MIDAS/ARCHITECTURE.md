# Architecture
This document describes the architecture of the MIDAS software flight code and surrounding ecosystem.

## Flight Code Structure
![Diagram of Flight Code](docs/architecture-diagram.png)

## Code Map
This section talks briefly about the code structure of the flight code for MIDAS, held under `/MIDAS`

The general architecture for the software is to have a central data struct that contains the current states and queues of data values. This data struct is the only way for threads to get information from other threads.

Speaking of threads: each different component will have its own thread. That is to say, each sensor, algorithim, data logger, gps, etc... will run on its own thread and communicate via the central data struct. For example, the barometer will be on its own thread sunning at its own speed, while telemetry will also have its own thread running at a slower speed.

The threads are split into two cores: the SENSOR_CORE (Core 0), and the DATA_CORE (Core 1). This is so that the slower operations, data logging, telemetry, etc..., do not slow down faster operations, like getting sensor data. This split also means after the rocket has landed we can shut off Core 0 to save battery but still have gps pings.
***

### `/src` 
All of the source code for the rocket

`main.cpp` Sets up the serial and rocket threads

`sensors.h` All of the definitions for each sensor and their functions. Each sensor is to be implemented in `/hardware`, `/hilsim`, and `/silsim` to be used in different applications

`systems.cpp` Starts all threads and constantly loops. Also contains a list of sensors and the rockcket_state struct for the data

`rocket_state.h` Central data struct which holds all of the current state adn queues for each sensor.

`hal.h` Macros to declare, start, and put to sleep different threads

`sensor_data.h` The type of data each sensor contains

`/hardware` Implementation for each of the pyhsical sensors on MIDAS

`/silsim` Implementation for SILSIM sensors. SILSIM is a software simulation program that tests flight code by simulating a flight on your computer.

`/hilsim` Implementation for HILSIM sensors. HILSIM is a program that tests out flight code on the physical hardware. The difference between this and `/hardware` is that sensor data is calculated on your computer and sent to the hardware to test in HILSIM, but is taken from actual sensors in `hardware`

***

### `/libs`
All of the libraries that are not in platformio that need to be used

***

`platformio.ini`  the build and upload configurations to use

`README.md` It's a readme, what do you expect