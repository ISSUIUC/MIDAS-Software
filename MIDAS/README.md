# MIDAS Flight Code

Flight software for the **MIDAS** avionics board. This repository contains the firmware, simulation environments, hardware drivers, telemetry, logging, and testing infrastructure used throughout development.

---

# Table of Contents

* [Overview](#overview)
* [Project Structure](#project-structure)
* [Requirements](#requirements)
* [Getting Started](#getting-started)
* [Building the Project](#building-the-project)
* [Running SILSIM](#running-silsim)
* [Uploading to Hardware](#uploading-to-hardware)
* [Testing](#testing)
* [Development](#development)
* [Future Documentation](#future-documentation)

---

# Overview

The MIDAS flight code is built using:

* **PlatformIO**
* **ESP32**
* **C++17**
* **FreeRTOS**

The repository contains everything needed to build, simulate, test, and deploy the rocket flight software.

The software is organized into independent modules such as the flight state machine, guidance/navigation, hardware drivers, telemetry, logging, and simulation.

---

# Project Structure

```
MIDAS/
│
├── lib/                # External and custom driver libraries
│
├── src/
│   ├── flight-systems/          # Main system initialization
│   ├── finite-state-machines/   # Flight state machines
│   ├── gnc/                     # Guidance, Navigation & Control
│   ├── hardware/                # Hardware interfaces
│   ├── hilsim/                  # Hardware-In-The-Loop simulation
│   ├── silsim/                  # Software-In-The-Loop simulation
│   ├── telemetry/               # Telemetry systems
│   ├── logging/                 # Data logging
│   ├── util/                    # Utility files that help
│   └── ...
│
├── test/               # Test programs and previous flight data
├── tools/              # Tools to help unload midas flight data (log_enc.py)
│
├── platformio.ini      # PlatformIO configuration
│
└── README.md
```

Additional modules will be documented individually as development progresses.

---

# Requirements

Before building the project, install:

* PlatformIO
* Git
* C++ compiler

For **SILSIM**, a native compiler is also required.

### Linux / macOS

`gcc` is typically already installed.

### Windows

Install the latest version of **MinGW-w64** and add its `bin` directory to your system `PATH`.

---

# Getting Started

Clone the repository:

```bash
git clone https://github.com/ISSUIUC/MIDAS-Software.git
cd MIDAS
```

More setup instructions will be added for:

* VSCode
* CLion
* PlatformIO installation

---

# Building the Project

## Build Flight Code

Compile the firmware without uploading:

```bash
pio run -e mcu_main
```

This verifies that the project builds successfully.

---

# Uploading to Hardware

Flash the firmware to the MIDAS board:

```bash
pio run -e mcu_main -t upload
```

Make sure the board is connected before uploading.

---

# Running SILSIM

To build and execute the Software-In-The-Loop simulator:

```bash
pio run -e mcu_silsim
```

SILSIM allows flight code to be tested without physical hardware.

> More documentation describing simulation inputs, outputs, and workflows will be added later.

---

# Testing

The `test/` directory contains:

* Previous flight datasets
* Test programs
* Validation utilities

Future documentation will include:

* Running unit tests
* Regression testing
* Flight replay testing
* Hardware validation

---

# Development

The project is divided into several major subsystems.

## Flight State Machines

Responsible for rocket state transitions throughout flight.

Future documentation:

* State diagrams
* Transition conditions
* Recovery logic

---

## Guidance, Navigation & Control (GNC)

Contains the vehicle estimation and control algorithms.

Future documentation:

* Sensor fusion
* Filtering
* Apogee detection
* Velocity estimation

---

## Hardware

Interfaces for onboard peripherals including sensors and communication devices.

Future documentation:

* Sensor drivers
* EEPROM
* SD card
* Radios
* Buzzer
* LEDs

---

## Telemetry

Handles communication between the flight computer and the ground station.

Future documentation:

* Packet formats
* Message types
* Protocol documentation

---

## Logging

Responsible for onboard data recording.

Future documentation:

* Log format
* Storage layout
* Replay tools

---

## SILSIM

Software-In-The-Loop simulator used during development.

Future documentation:

* Flight replay
* Sensor injection
* Configuration
* Debugging

---

## HILSIM

Hardware-In-The-Loop testing framework.

Future documentation:

* Hardware setup
* Test workflow
* Communication protocol

---

# Coding Style

Documentation for coding conventions, formatting, and contribution guidelines will be added in the future.

---

# Future Documentation

Planned additions include:

* PlatformIO installation guide
* VSCode setup
* CLion setup
* Project architecture
* State machine documentation
* Sensor documentation
* Telemetry protocol
* Logging format
* Build environments
* Flashing multiple boards
* Debugging with PlatformIO
* Simulation guide
* Contribution guidelines
* Continuous Integration (CI)
* Flight data analysis

---

# License

*To be added.*
