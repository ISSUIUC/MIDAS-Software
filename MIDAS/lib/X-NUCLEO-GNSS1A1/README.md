# X-NUCLEO-GNSS1A1
Arduino library to support the X-NUCLEO-GNSS1A1 expansion board using the TESEO-LIV3F module

## API

This device uses alternatively I2C or UART to communicate. An I2C or UART instance is required to access the device.
  
## Examples

There are 5 examples with the  X-NUCLEO-GNSS1A1 library.

* X_NUCLEO_GNSS1A1_HelloWorld_I2C: This example code provides a simple command line interface
  to communicate with the sensor via I2C protocol

* X_NUCLEO_GNSS1A1_HelloWorld_UART: This example code provides a simple command line interface
  to communicate with the sensor via UART protocol

* X_NUCLEO_GNSS1A1_MicroNMEA_I2C: This example code shows how to communicate with the sensor via 
  I2C protocol using the lightweight Arduino MicroNMEA library. 

* X_NUCLEO_GNSS1A1_MicroNMEA_UART: This example code shows how to communicate with the sensor via 
  UART protocol using the lightweight Arduino MicroNMEA library. 

* X_NUCLEO_GNSS1A1_VirtualCOMPort: This example code should be uploaded to the board in order to perform a
  firmware upgrade using the Flash Updater java application. 

## Dependencies

The X-NUCLEO-GNSS1A1 library requires the following Arduino library:

* MicroNMEA: https://github.com/stevemarple/MicroNMEA

In order to perform the firmware upgrade, the following Java application should be used:

* Flash Updater: https://github.com/stm32duino/Teseo-LIV3F-Flash-Updater

## Note

The device works only in an outdoor enviroment with a clear view of the sky.
In order to prevent data loss the update function used in the HelloWorld examples should be called at least 20 times per second.
Both the HelloWorld examples need at least 16KB of RAM in order to work properly. The RAM requirements for the MicroNMEA examples
depend on the board architecture and can vary from 1KB to 3KB.

## Documentation

You can find the source files at  
https://github.com/stm32duino/X-NUCLEO-GNSS1A1

The TESEO-LIV3F datasheet is available at  
https://www.st.com/content/st_com/en/products/positioning/gnss-modules/teseo-liv3f.html
