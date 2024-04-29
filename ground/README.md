# GSS v1.1
This is the primary source of information for all things related to **GSS 1.1**, the telemetry system used for Spaceshot's 2024 launch of *Kairos II* at FAR 51025!

## Contributors

- Nicholas Phillips
- Gautam Dayal
- Patrick Marschoun
- Peter Giannetos
- Aaditya Voruganti
- Michael Karpov

System architecture:

Changes from **GSS 1.0**: Unlike GSS 1.0, the new system uses a single laptop to combine streams from our antenna array and send out the data to a network, with all other data consumers subscribing to a stream over [MQTT](https://mqtt.org/). Additionally, this architecture allows us to implement other systems into our ecosystem which use telemetry data but would otherwise slow down our telemetry computers.

![System Architecture](https://i.ibb.co/YtWs14w/Screenshot-2024-04-16-190354.png)

# Telemetry Recievers
This section refers to the Feather Reciever software which is stored at `./src/feather`.

This repository contains the software for the ground station hardware (LoRa Feather module) used by the Spaceshot Telemetry Team during the 2023-24 season.

## Description

The ground station software is responsible for receiving telemetry data from the rocket, processing commands sent from the ground station GUI, and interfacing with the LoRa Feather module. It includes functionalities such as setting frequency, sending commands to the rocket, and parsing telemetry data.

## Features

- Receive telemetry data from the rocket
- Send commands to the rocket
- Interface with the ground station GUI through serial communication
- Parse incoming commands from the ground station GUI
- Set frequency for communication with the rocket
- Decode and process telemetry data packets
- Handle errors and retries in command transmission

## Dependencies

- [RH_RF95 library](https://github.com/PaulStoffregen/RadioHead) - for LoRa communication
- [SPI library](https://github.com/PaulStoffregen/SPI) - for SPI communication
- [SerialParser library](link-to-serial-parser-library) - for parsing serial input

## Usage

1. Connect the LoRa Feather module to the ground station hardware.
2. Upload the appropriate version of the code to the ground station hardware using the Arduino IDE or compatible software.
   - **For Ground Station**: Upload the code with `IS_GROUND` defined.
   - **For Drone**: Upload the code with `IS_DRONE` defined.
3. Open the serial monitor to view output messages and interact with the ground station GUI.
4. Follow the commands and instructions provided by the ground station GUI to control the rocket and receive telemetry data.

## Configuration

- Adjust default frequencies (`RF95_FREQ`, `SUSTAINER_FREQ`, `BOOSTER_FREQ`, `GROUND_FREQ`) as needed for your application.
- RF95_FREQ is used only when `IS_GROUND` is active
- `SUSTAINER_FREQ`, `BOOSTER_FREQ`, `GROUND_FREQ` is used only when `IS_DRONE` is active

## Flashing Instructions

### Ground Station

It is used for the typical feathers on the ground and will be able to receive any code either directly from the rocket or from the drone relay.

1. Connect the ground station hardware to your computer.
2. Open the Arduino IDE or compatible software.
3. Load the code with `IS_GROUND` defined.
4. Compile and upload the code to the ground station hardware.

### Drone

This code is used by feathers to relay information via a drone from the rocket to ground, this is for better connectivity when the rocket is further away from us or is covered by earth elements.

1. Connect the drone hardware to your computer.
2. Open the Arduino IDE or compatible software.
3. Load the code with `IS_DRONE` defined.
4. Compile and upload the code to the drone hardware.







To upload to the reciever you must navigate to the `ground` directory and use the `Platformio` vscode extension and upload using either the `feather` or `drone` build environments.
Alternatively use the `pio` command like so:

```bash
$ pio run -t upload -e <feather/drone>
```

# Ground Station Combiner
This section refers to the Ground Station Combiner software which is stored at `./gss_combiner`.
## Installation and Operation
First, begin by cloning the repository to your computer (or opening it) and opening the `gss_combiner` folder:

```bash
$ git clone https://github.com/ISSUIUC/MIDAS-Software.git
$ cd ./MIDAS-Software/ground/gss_combiner
```

To install dependencies for the combiner, you can run:
```bash
$ pip install -r requirements.txt
```

*(MK) TODO: Add requirements.txt file.*

Then, you can run the combiner using **Python**:

```bash
$ python ./main.py <options>
```

`<options>` refers to a set of command line arguments that can be passed to `main.py`, listed below:

`--booster <COM1,COM2,COM3,...>`: Pass in a comma-separated list of COM ports to receive telemetry from, and transmit to the `Booster` data topics. 

`--sustainer <COM1,COM2,COM3,...>`: Pass in a comma-separated list of COM ports to receive telemetry from, and transmit to the `Sustainer` data topics. 

`--relay <COM1,COM2,COM3,...>`: Pass in a comma-separated list of COM ports to receive telemetry from, specifically for the `Drone Relay` system.

`--local (alias -l)`: Use `localhost` as the MQTT target. Useful for debugging.

`--no-log (alias -n)`: Do not generate logs for this run.

`--verbose (alias -v)`: Print out all combiner actions. This may slow down the combiner due to print volume.

`--no-vis (alias -nv)`:  Disable the visualization for system health (Also disabled with `--verbose`).

`--config <config> (alias -c)`: Load an argument configuration from the `config.ini` file

`--no-rf`: Skip overriding RF frequencies for the feather reciever.

`--help (alias -h)`: Display a set of these options.


Not including sustainer / booster sources will throw a warning, but will still run the system, allowing you to check the connectivity for the backend MQTT broker.

### For Kairos II Summer Launch 2024:
The `config.ini` file will be updated to include all necessary configuration within the `launch` config. As such you will only need to edit the COM ports present in the file, and you will be able to execute the system with the command

```bash
$ python ./main.py -c launch
```


Additionally, for this launch we have adopted the following lookup scheme for determining the stage callsign:

| Callsign bit value (highest bit of `fsm_callsign_satcount`) | Callsign |
| ----------------------------------------------------------- | -------- |
| 0                                                           | KD9ZPM   |
| 1                                                           | KD9ZMJ   |


## MQTT Streams
This system uses [MQTT](https://mqtt.org/) as the primary data transfer method to other sections of the telemetry system. This is accomplished using multiple data streams. While technically unsecured, `data` streams are intended to be read-only (and only written to by this service), while `control` streams are intended to allow control of the system. As of writing (4/16), the current accepted data streams for GSS 1.1 are the following:

`FlightData-All` (`data`): Subscribe to receive all telemetry packets (Both booster and sustainer)

`FlightData-Sustainer` (`data`): Subscribe to receive sustainer telemetry packets

`FlightData-Booster` (`data`): Subscribe to receive booster telemetry packets

`Control-Sustainer` (`control`): Publish to edit Sustainer telemetry system functionality.

`Control-Booster` (`control`): Publish to edit Booster telemetry system functionality.

`Common` (`control / data`): Data published by auxiliary services or non-critical systems. 


## Null-Modem Emulation
It is possible to run full tests of this telemetry system without access to the telemetry hardware. Installing a null-modem emulator such as [this one](https://com0com.sourceforge.net/) will allow you to emulate COM ports on your device and run the `test/test.py` script. 

While this is the software we use for internal testing of this system, we cannot guarantee your results regarding installation and software safety.

