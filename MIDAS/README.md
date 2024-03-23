# MIDAS Flight Code

## Getting Started

To being, clone this whole repository onto your computer somewhere.

// todo explain how to install/use platformio
// todo have both VSCode and CLion sections

### Command Line
If you want to run SILSIM, you'll need `gcc` to be available on your path. 
On Linux/macOS, `gcc` should be preinstalled. 
On Windows, you'll need to install the newest version of MinGW-w64 and add it's
`\bin` directory to your PATH environment variable.

To flash flight code to the MIDAS board using the command line, run
`pio run -e mcu_main -t upload`. To just build flight code for MIDAS without
actually uploading it (to make sure it compiles), use `pio run -e mcu_main`.
Note that if you never directly installed platformio and instead are just using
the VSCode extension, these commands won't work until you install platformio
manually.

To run SILSIM from the command line, use `pio run -e mcu_silsim`.