# GSS v1.1
This is the primary source of information for all things related to **GSS 1.1**, the telemetry system used for Spaceshot's 2024 launch of *Kairos II* at FAR 51025!

System architecture:


Changes from **GSS 1.0**: Unlike GSS 1.0, the new system uses a single laptop to combine streams from our antenna array and send out the data to a network, with all other data consumers subscribing to a stream over [MQTT](https://mqtt.org/). Additionally, this architecture allows us to implement other systems into our ecosystem which use telemetry data but would otherwise slow down our telemetry computers.

![System Architecture](https://i.ibb.co/YtWs14w/Screenshot-2024-04-16-190354.png)

## Installation and Operation
This repository contains code specific to the **MQTT Telemetry Combiner**.
First, begin by cloning the repository to your computer and opening the `gss_combiner` folder:

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

`--local (alias -l)`: Use `localhost` as the MQTT target. Useful for debugging.

`--no-log (alias -n)`: Do not generate logs for this run.

`--verbose (alias -v)`: Print out all combiner actions. This may slow down the combiner due to print volume.

`--no-vis (alias -nv)`:  Disable the visualization for system health (Also disabled with `-v`).

`--help (alias -h)`: Display a set of these options.


Not including sustainer / booster sources will throw a warning, but will still run the system, allowing you to check the connectivity for the backend MQTT broker.

*(MK) Note: This repository also contains Mappy code.. we should probably move it to the Ground-Station-App repository*
## MQTT Streams
This system uses [MQTT](https://mqtt.org/) as the primary data transfer method to other sections of the telemetry system. This is accomplished using multiple data streams. While technically unsecured, `data` streams are intended to be read-only (and only written to by this service), while `control` streams are intended to allow control of the system. As of writing (4/16), the current accepted data streams for GSS 1.1 are the following:

`FlightData-All` (`data`): Subscribe to receive all telemetry packets (Both booster and sustainer)
`FlightData-Sustainer` (`data`): Subscribe to receive sustainer telemetry packets
`FlightData-Booster` (`data`): Subscribe to receive booster telemetry packets
`Control-Sustainer` (`control`): Publish to edit Sustainer telemetry system functionality.
`Control-Booster` (`control`): Publish to edit Booster telemetry system functionality.
`Common` (`control / data`): Data published by auxiliary services or non-critical systems. 

