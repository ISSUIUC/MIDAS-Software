/**
 ******************************************************************************
 * @file    X_NUCLEO_GNSS1A1_HelloWorld_UART.ino
 * @author  AST
 * @version V1.0.0
 * @date    January 2018
 * @brief   Arduino test application for the STMicrolectronics X-NUCLEO-GNSS1A1
 *          GNSS module expansion board based on TeseoLIV3F.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

//NOTE: for compatibility with the Arduino Due some additional cabling needs to be performed:
//      pin D8 should be connected to pin D18 and pin D2 should be connected to pin D19

#include "teseo_liv3f_class.h"

TeseoLIV3F *gps;
int incomingByte;
GNSSParser_Data_t data;
char command[32] = {0};
char msg[256];
int cmdType = 0;
uint32_t t, s;
int tracked;
GPGGA_Info_t stored_positions[64];
int status = 0;
uint32_t stime = 0;
int waitType = 0;

#ifdef ARDUINO_ARCH_STM32
HardwareSerial Serial1(PA10, PA9);
#endif

#define MSG_SZ 256
#define waitForRequest 0
#define waitForAnswer 1

#if defined(ARDUINO_SAM_DUE)
#include <avr/dtostrf.h>
#elif defined(ARDUINO_ARCH_STM32L0)
#include <avr/dtostrf.h>
#endif


void setup()
{
   //Initialize serial port for user communication
   Serial.begin(115200);
   //Initialize serial port for device communication
   Serial1.begin(9600);
   Serial.println("Setup begin");
   //Create the device object passing to it the serial interface
   gps = new TeseoLIV3F(&Serial1, 7, 13);
   //Initialize the device
   gps->init();
   Serial.println("Setup end");
   //Show list of commands
   showCmds();
}

void loop()
{
   //recieve data form the serial port
   //Note: this function shoud be called AT LEAST 20 times per second
   //      your code should not have any blocking function in order to prevent data loss
   gps->update();

   //if the board is waiting for a specific message to arrive
   if (status == waitForAnswer)
   {
      waitResponse();
   }

   //if waiting for user command
   if (status == waitForRequest)
   {
      parseUserCommand();
   }
}


void parseUserCommand ()
{
   //if the user wrote something
   if (Serial.available() > 0)
   {
      // read the oldest byte in the serial buffer:
      incomingByte = Serial.read();

      //if the char is valid write it to terminal
      if (incomingByte> 31 && incomingByte < 126)
      {
         Serial.print((char)incomingByte);
      }

      //if is fetched a carriage return
      if (incomingByte == '\r')
      {
         Serial.print("\r\n");

         //if the command isn't an empty string
         if (strlen(command) > 0)
         {
            //add the null terminator and parse it
            command[strlen(command)] = '\0';
            AppCmdProcess(command);
            memset (command, 0, sizeof(command));
         }
         else
            showPrompt();
      }
      //add the incoming character to the buffer if valid
      else if (incomingByte > 31 && incomingByte < 126)
      {
         command[strlen(command)] = incomingByte;
      }
   }
}


void waitResponse ()
{
   //ask the device if the message has arrived
   int result = gps->getMessageDone();
   //if it arrived
   if (result)
   {
      Serial.println("OK");
      //if it was a version message
      if (waitType==1)
      {
         //print the version infos
         getPSTMVERInfo();
         //wait for user interaction
         waitType=0;
         status = waitForRequest;
      }
      //if it was a configuration message
      else if (waitType==2)
      {
         //get the data
         data = gps->getData();
         //if the command was a success
         if (data.result == GNSS_OP_OK)
         {
            //delay in order to prevent i2c issues
            delay(500);
            Serial.print("Saving NMEA msg configuration...\t");
            Serial.print("\r\n>");
            //send the message to save the new parameters and start waiting
            gps->sendCommand((char *)"$PSTMSAVEPAR");
            gps->askMessage((char *)"$PSTMSAVEPAR");
            waitType=3;
            stime = millis();
         }
         //if it failed reset
         else
         {
            waitType=0;
            status = waitForRequest;
         }
      }
      //if it was a save message
      else if (waitType==3)
      {
         //get the data
         data = gps->getData();
         //if the command was a success
         if (data.result == GNSS_OP_OK)
         {
            //reset the sensor
            Serial.print(" Resetting...\t");
            gps->init();
            Serial.print("\r\n>");
         }
         //wait for user interaction
         waitType=0;
         status = waitForRequest;
      }
   }
   //if the timer expired (10s)
   else if (millis() > (stime + 10000))
   {
      //wait for user interaction
      Serial.print("Timed out\r\n>");
      waitType=0;
      status = waitForRequest;
   }
}


void AppCmdProcess (char *cmd)
{
   //if it's a standard command
   if (cmdType == 0)
   {
      if((strcmp((char *)cmd, "1") == 0 || strcmp((char *)cmd, "getpos") == 0) ||
            (strcmp((char *)cmd, "2") == 0 || strcmp((char *)cmd, "lastpos") == 0))
      {
         //print the current gpgga values
         printValidInfo();
      }
      else if (strcmp((char *)cmd, "3") == 0 || strcmp((char *)cmd, "wakestatus") == 0)
      {
         //print the current device status
         Serial.print("WakeUp Status: ");
         gps->getWakeupStatus() == 0 ? Serial.print("0") : Serial.print("1");
         Serial.print("\r\n>");
      }
      else if(strcmp((char *)cmd, "4") == 0 || strcmp((char *)cmd, "help") == 0)
      {
         //show the commands again
         showCmds();
      }
      else if(strcmp((char *)cmd, "5") == 0 || strcmp((char *)cmd, "debug") == 0)
      {
         //activate/deactivate debug prints
         Debug_State state = gps->toggleDebug();
         if(state == DEBUG_OFF)
            Serial.print("Debug: OFF\r\n>");
         else
            Serial.print("Debug: ON\r\n>");
      }
      else if(strcmp((char *)cmd, "6") == 0 || strcmp((char *)cmd, "track") == 0)
      {
         //ask the number of positions to track and wait for response
         sprintf(msg, "How many positions do you want to track? (max allowed %d)\r\n>", 64);
         Serial.print(msg);
         cmdType=1;
      }
      else if(strcmp((char *)cmd, "7") == 0 || strcmp((char *)cmd, "lasttrack") == 0)
      {
         //print the list of tracked positions
         if(tracked > 0)
         {
            Serial.print("Acquired positions:\r\n");
            printTrackedPositions(tracked);
         }
         else
            Serial.print("Last tracking process went bad.\r\n\n>");
      }
      else if(strcmp((char *)cmd, "8") == 0 || strcmp((char *)cmd, "getfwver") == 0)
      {
         //print the version command list
         Serial.print("Type \"$PSTMGETSWVER\"   to get the GNSSLIB version \r\n");
         Serial.print("Type \"$PSTMGETSWVER,1\" to get the OS20LIB version \r\n");
         Serial.print("Type \"$PSTMGETSWVER,2\" to get the GPSAPP version \r\n");
         Serial.print("Type \"$PSTMGETSWVER,4\" to get the WAASLIB version \r\n");
         Serial.print("Type \"$PSTMGETSWVER,6\" to get the BINIMG version \r\n");
         Serial.print("Type \"$PSTMGETSWVER,7\" to get the board version \r\n");
         Serial.print("Type \"$PSTMGETSWVER,8\" to get the STAGPSLIB version \r\n");
         Serial.print("\nType the command now:\r\n> ");
      }
      else if(strncmp((char *)cmd, "$PSTMGETSWVER", strlen("$PSTMGETSWVER")) == 0)
      {
         //send the version command and wait for response
         gps->sendCommand(cmd);
         gps->askMessage((char *)"$PSTMVER");
         status = waitForAnswer;
         waitType=1;
         stime = millis();
      }
      else if(strcmp((char *)cmd, "9") == 0 || strcmp((char *)cmd, "getgnsmsg") == 0)
      {
         //print --gns informations
         getGNSInfo();
      }
      else if(strcmp((char *)cmd, "10") == 0 || strcmp((char *)cmd, "getgpgst") == 0)
      {
         //print gpgst informations
         getGPGSTInfo();
      }
      else if(strcmp((char *)cmd, "11") == 0 || strcmp((char *)cmd, "getgprmc") == 0)
      {
         //print gprmc informations
         getGPRMCInfo();
      }
      else if(strcmp((char *)cmd, "12") == 0 || strcmp((char *)cmd, "getgsamsg") == 0)
      {
         //print --gsa informations
         getGSAInfo();
      }
      else if(strcmp((char *)cmd, "13") == 0 || strcmp((char *)cmd, "getgsvmsg") == 0)
      {
         //print --gsv informations
         getGSVInfo();
      }
      else if(strcmp((char *)cmd, "19") == 0 || strcmp((char *)cmd, "ext-help") == 0)
      {
         //print the extended help
         printHelp();
      }
      else if(strcmp((char *)cmd, "y") == 0)
      {
         //send the command for the message list and wait for response
         int lowMask = 0x18004F;

         char gnssCmd[100];

         snprintf(gnssCmd, 90, "$PSTMCFGMSGL,%d,%d,%x,%x",
                  0, /*NMEA 0*/
                  1, /*Rate*/
                  lowMask,
                  0x0);

         gps->sendCommand(gnssCmd);

         gps->askMessage((char *)"$PSTMCFGMSGL");
         status = waitForAnswer;
         waitType=2;
         stime = millis();
         Serial.print("\r\n>");
      }
      else if(strcmp((char *)cmd, "n") == 0)
      {
         //do nothing, show prompt again
         Serial.print("\r\n>");
      }
      else
      {
         //the command inserted is malformed
         Serial.print("Command not valid.\r\n\n>");
      }
   }
   //if waiting for the number of positions to track
   else if (cmdType == 1)
   {
      //if the input is wrong ask again
      if (atoi((char *)cmd) < 0 || atoi((char *)cmd) > 64)
      {
         sprintf(msg, "How many positions do you want to track? (max allowed %d)\r\n>", 64);
         Serial.print(msg);
      }
      //ask the time interval between the misurations
      else
      {
         t = strtoul((char *)cmd, NULL, 10);
         Serial.print("How many seconds do you want to delay while tracking? (>= 0)\r\n> ");
         cmdType = 2;
      }
   }
   //if waiting for time interval
   else if (cmdType == 2)
   {
      //if the input is wrong keep asking
      if (atoi((char *)cmd) < 0)
      {
         Serial.print("How many seconds do you want to delay while tracking? (>= 0)\r\n> ");
      }
      else
      {
         //start tracking positions
         s = strtoul((char *)cmd, NULL, 10);
         tracked = trackGotPos(t, s);
         //if it tracked at least one position
         if(tracked > 0)
         {
            Serial.print("Last tracking process went good.\r\n\n>");
         }
         else
            Serial.print("Last tracking process went bad.\r\n\n>");
         cmdType=0;
      }
   }
}

void showCmds()
{
   Serial.print("Select a command:\r\n");
   Serial.print(" 1 - getpos\r\n");
   Serial.print(" 2 - lastpos\r\n");
   Serial.print(" 3 - wakestatus\r\n");
   Serial.print(" 4 - help\r\n");
   Serial.print(" 5 - debug\r\n");
   Serial.print(" 6 - track\r\n");
   Serial.print(" 7 - lasttrack\r\n");
   Serial.print(" 8 - getfwver\r\n");
   Serial.print(" 9 - getgnsmsg\r\n");
   Serial.print("10 - getgpgst\r\n");
   Serial.print("11 - getgprmc\r\n");
   Serial.print("12 - getgsamsg\r\n");
   Serial.print("13 - getgsvmsg\r\n");
   Serial.print("19 - ext-help\r\n");
   Serial.print("\r\nSave configuration (y/n)? ");
   Serial.print("\r\n> ");
}

void showPrompt()
{
   Serial.print("> ");
}

void printValidInfo()
{
   data = gps->getData();
   if (data.gpgga_data.valid == 1)
   {
      int lat = (int) (data.gpgga_data.xyz.lat/100.0);
      int lat_mod = (int) (data.gpgga_data.xyz.lat) - (lat * 100);
      int lon = (int) (data.gpgga_data.xyz.lon/100.0);
      int lon_mod = (int) (data.gpgga_data.xyz.lon) - (lon * 100);
      char alt[7];
      char acc[5];
      dtostrf (data.gpgga_data.xyz.alt, 3, 2, alt);
      dtostrf (data.gpgga_data.acc, 4, 1, acc);
      snprintf(msg, MSG_SZ, "UTC:\t\t\t[ %02ld:%02ld:%02ld ]\r\n",
               data.gpgga_data.utc.hh,
               data.gpgga_data.utc.mm,
               data.gpgga_data.utc.ss);
      Serial.print(msg);

      snprintf(msg, MSG_SZ, "Latitude:\t\t[ %.02d' %.02d'' %c ]\r\n",
               lat,
               lat_mod,
               data.gpgga_data.xyz.ns);
      Serial.print(msg);

      snprintf(msg, MSG_SZ, "Longitude:\t\t[ %.02d' %.02d'' %c ]\r\n",
               lon,
               lon_mod,
               data.gpgga_data.xyz.ew);
      Serial.print(msg);

      snprintf(msg, MSG_SZ, "Satellites locked:\t[ %ld ]\r\n",
               data.gpgga_data.sats);
      Serial.print(msg);

      snprintf(msg, MSG_SZ, "Position accuracy:\t[ %s ]\r\n",
               acc);
      Serial.print(msg);

      snprintf(msg, MSG_SZ, "Altitude:\t\t[ %s%c ]\r\n",
               alt,
               (data.gpgga_data.xyz.mis + 32U));
      Serial.print(msg);

      snprintf(msg, MSG_SZ, "Geoid infos:\t\t[ %ld%c ]\r\n",
               data.gpgga_data.geoid.height,
               data.gpgga_data.geoid.mis);
      Serial.print(msg);

      snprintf(msg, MSG_SZ, "Diff update:\t\t[ %ld ]\r\n",
               data.gpgga_data.update);
      Serial.print(msg);
   }
   else
   {
      Serial.print("Last position wasn't valid.\r\n\n");
   }
   Serial.print("\r\n\n>");
}



void getGNSInfo()
{
   data = gps->getData();
   Serial.print("\r\n");

   snprintf(msg, MSG_SZ,  "Constellation:\t\t[ %s ]\r\n",
            data.gns_data.constellation);
   Serial.print(msg);

   if (strcmp((char*)data.gns_data.constellation, "$GPGNS") == 0)
   {
      Serial.print("-- only GPS constellation is enabled\r\n");
   }
   else if (strcmp((char*)data.gns_data.constellation, "$GLGNS") == 0)
   {
      Serial.print("-- only GLONASS constellation is enabled\r\n");
   }
   else if (strcmp((char*)data.gns_data.constellation, "$GAGNS") == 0)
   {
      Serial.print("-- only GALILEO constellation is enabled\r\n");
   }
   else if (strcmp((char*)data.gns_data.constellation, "$BDGNS") == 0)
   {
      Serial.print("-- only BEIDOU constellation is enabled\r\n");
   }
   else if (strcmp((char*)data.gns_data.constellation, "$QZGNS") == 0)
   {
      Serial.print("-- only QZSS constellation is enabled\r\n");
   }
   else if (strcmp((char*)data.gns_data.constellation, "$GNGSV") == 0)
   {
      Serial.print("-- message to report all satellites for all enabled constellations\r\n");
   }
   else
   {
      /* nothing to do */
   }

   int lat = (int) (data.gns_data.xyz.lat/100.0);
   int lat_mod = (int) (data.gns_data.xyz.lat) - (lat * 100);
   int lon = (int) (data.gns_data.xyz.lon/100.0);
   int lon_mod = (int) (data.gns_data.xyz.lon) - (lon * 100);
   char alt[7];
   char hdop[6];
   char geoid[5];
   dtostrf (data.gns_data.xyz.alt, 2, 2, alt);
   dtostrf (data.gns_data.hdop, 2, 1, hdop);
   dtostrf (data.gns_data.geo_sep, 2, 1, geoid);

   snprintf(msg, MSG_SZ,  "UTC:\t\t\t[ %02ld:%02ld:%02ld ]\r\n",
            data.gns_data.utc.hh,
            data.gns_data.utc.mm,
            data.gns_data.utc.ss);
   Serial.print(msg);

   snprintf(msg, MSG_SZ, "Latitude:\t\t[ %d' %d'' %c ]\r\n",
            lat,
            lat_mod,
            data.gns_data.xyz.ns);
   Serial.print(msg);

   snprintf(msg, MSG_SZ, "Longitude:\t\t[ %d' %d'' %c ]\r\n",
            lon,
            lon_mod,
            data.gns_data.xyz.ew);
   Serial.print(msg);

   snprintf(msg, MSG_SZ, "Satellites locked:\t[ %ld ]\r\n",
            data.gns_data.sats);
   Serial.print(msg);

   snprintf(msg, MSG_SZ, "HDOP:\t\t\t[ %s ]\r\n",
            hdop);
   Serial.print(msg);

   snprintf(msg, MSG_SZ, "Altitude:\t\t[ %s ]\r\n",
            alt);
   Serial.print(msg);

   snprintf(msg, MSG_SZ, "Geoid infos:\t\t[ %s ]\r\n",
            geoid);
   Serial.print(msg);

   Serial.print("\r\n\n>");
}


void getGPGSTInfo()
{
   data = gps->getData();
   Serial.print("\r\n");

   char ephe[6];
   char majdev[6];
   char mindev[6];
   char majang[6];
   char laterr[6];
   char lonerr[6];
   char alterr[6];
   dtostrf (data.gpgst_data.EHPE, 2, 1, ephe);
   dtostrf (data.gpgst_data.semi_major_dev, 2, 1, majdev);
   dtostrf (data.gpgst_data.semi_minor_dev, 2, 1, mindev);
   dtostrf (data.gpgst_data.semi_major_angle, 2, 1, majang);
   dtostrf (data.gpgst_data.lat_err_dev, 2, 1, laterr);
   dtostrf (data.gpgst_data.lon_err_dev, 2, 1, lonerr);
   dtostrf (data.gpgst_data.alt_err_dev, 2, 1, alterr);

   snprintf(msg, MSG_SZ,  "UTC:\t\t\t[ %02ld:%02ld:%02ld ]\r\n",
            data.gpgst_data.utc.hh,
            data.gpgst_data.utc.mm,
            data.gpgst_data.utc.ss);
   Serial.print(msg);

   snprintf(msg, MSG_SZ, "EHPE:\t\t\t[ %s ]\r\n",
            ephe);
   Serial.print(msg);

   snprintf(msg, MSG_SZ, "Semi-major Dev:\t\t[ %s ]\r\n",
            majdev);
   Serial.print(msg);

   snprintf(msg, MSG_SZ, "Semi-minor Dev:\t\t[ %s ]\r\n",
            mindev);
   Serial.print(msg);

   snprintf(msg, MSG_SZ, "Semi-maior Angle:\t[ %s ]\r\n",
            majang);
   Serial.print(msg);

   snprintf(msg, MSG_SZ, "Lat Err Dev:\t\t[ %s ]\r\n",
            laterr);
   Serial.print(msg);

   snprintf(msg, MSG_SZ, "Lon Err Dev:\t\t[ %s ]\r\n",
            lonerr);
   Serial.print(msg);

   snprintf(msg, MSG_SZ, "Alt Err Dev:\t\t[ %s ]\r\n",
            alterr);
   Serial.print(msg);

   Serial.print("\r\n\n>");

   return;
}

void getGPRMCInfo()
{
   data = gps->getData();
   Serial.print("\r\n");

   snprintf(msg, MSG_SZ,  "UTC:\t\t\t\t[ %02ld:%02ld:%02ld ]\r\n",
            data.gprmc_data.utc.hh,
            data.gprmc_data.utc.mm,
            data.gprmc_data.utc.ss);
   Serial.print(msg);

   snprintf(msg, MSG_SZ, "Status:\t\t\t\t[ %c ]\t\t",
            data.gprmc_data.status);
   Serial.print(msg);
   if (data.gprmc_data.status == (uint8_t)'A')
   {
      Serial.print("-- Valid (reported in 2D and 3D fix conditions)\r\n");
   }
   else if (data.gprmc_data.status == (uint8_t)'V')
   {
      Serial.print("-- Warning (reported in NO FIX conditions)\r\n");
   }
   else
   {
      Serial.print("-- Unknown status\r\n");
   }

   int lat = (int) (data.gprmc_data.xyz.lat/100.0);
   int lat_mod = (int) (data.gprmc_data.xyz.lat) - (lat * 100);
   int lon = (int) (data.gprmc_data.xyz.lon/100.0);
   int lon_mod = (int) (data.gprmc_data.xyz.lon) - (lon * 100);
   char speed[6];
   char trackgood[6];
   char mag_var[6];
   dtostrf (data.gprmc_data.speed, 2, 1, speed);
   dtostrf (data.gprmc_data.trackgood, 2, 1, trackgood);
   dtostrf (data.gprmc_data.mag_var, 2, 1, mag_var);

   snprintf(msg, MSG_SZ, "Latitude:\t\t\t[ %d' %02d'' %c ]\r\n",
            lat,
            lat_mod,
            data.gprmc_data.xyz.ns);
   Serial.print(msg);

   snprintf(msg, MSG_SZ, "Longitude:\t\t\t[ %d' %02d'' %c ]\r\n",
            lon,
            lon_mod,
            data.gprmc_data.xyz.ew);
   Serial.print(msg);

   snprintf(msg, MSG_SZ, "Speed over ground (knots):\t[ %s ]\r\n",
            speed);
   Serial.print(msg);

   snprintf(msg, MSG_SZ, "Trackgood:\t\t\t[ %s ]\r\n",
            trackgood);
   Serial.print(msg);

   snprintf(msg, MSG_SZ, "Date (ddmmyy):\t\t\t[ %ld ]\r\n",
            data.gprmc_data.date);
   Serial.print(msg);

   snprintf(msg, MSG_SZ, "Magnetic Variation:\t\t[ %s ]\r\n",
            mag_var);
   Serial.print(msg);

   if ((data.gprmc_data.mag_var_dir != (uint8_t)'E') &&
         (data.gprmc_data.mag_var_dir != (uint8_t)'W'))
   {
      snprintf(msg, MSG_SZ, "Magnetic Var. Direction:\t[ - ]\r\n");
   }
   else
   {
      snprintf(msg, MSG_SZ, "Magnetic Var. Direction:\t[ %c ]\r\n",
               data.gprmc_data.mag_var_dir);
   }
   Serial.print(msg);

   Serial.print("\r\n\n>");

   return;
}

void getGSAInfo()
{
   data = gps->getData();
   Serial.print("\r\n");

   snprintf(msg, MSG_SZ,  "Constellation:\t\t[ %s ]\t",
            data.gsa_data.constellation);
   Serial.print(msg);

   if (strcmp((char*)data.gsa_data.constellation, "$GPGSA") == 0)
   {
      Serial.print("-- only GPS constellation is enabled\r\n");
   }
   else if (strcmp((char*)data.gsa_data.constellation, "$GLGSA") == 0)
   {
      Serial.print("-- only GLONASS constellation is enabled\r\n");
   }
   else if (strcmp((char*)data.gsa_data.constellation, "$GAGSA") == 0)
   {
      Serial.print("-- only GALILEO constellation is enabled\r\n");
   }
   else if (strcmp((char*)data.gsa_data.constellation, "$BDGSA") == 0)
   {
      Serial.print("-- only BEIDOU constellation is enabled\r\n");
   }
   else if (strcmp((char*)data.gsa_data.constellation, "$GNGSA") == 0)
   {
      Serial.print("-- more than one constellation is enabled\r\n");
   }
   else
   {
      /* nothing to do */
   }

   snprintf(msg, MSG_SZ,  "Operating Mode:\t\t[ %c ]\t\t",
            data.gsa_data.operating_mode);
   Serial.print(msg);

   if (data.gsa_data.operating_mode == (uint8_t)'A')
   {
      Serial.print("-- Auto (2D/3D)\r\n");
   }
   else if (data.gsa_data.operating_mode == (uint8_t)'M')
   {
      Serial.print("-- Manual\r\n");
   }
   else
   {
      Serial.print("-- Unknown op mode\r\n");
   }

   snprintf(msg, MSG_SZ, "Current Mode:\t\t[ %ld ]\t\t",
            data.gsa_data.current_mode);
   Serial.print(msg);

   if (data.gsa_data.current_mode == 1)
   {
      Serial.print("-- no fix available\r\n");
   }
   else if (data.gsa_data.current_mode == 2)
   {
      Serial.print("-- 2D\r\n");
   }
   else if (data.gsa_data.current_mode == 3)
   {
      Serial.print("-- 3D\r\n");
   }
   else
   {
      /* nothing to do */
   }

   int32_t *sat_prn = data.gsa_data.sat_prn;
   for (uint8_t i=0; i<12U; i++)
   {
      snprintf(msg, MSG_SZ, "SatPRN%02d:\t\t[ %ld ]\r\n", i+1U,
               *(&sat_prn[i]));
      Serial.print(msg);
   }

   char pdop[6];
   char hdop[6];
   char vdop[6];
   dtostrf (data.gsa_data.pdop, 2, 1, pdop);
   dtostrf (data.gsa_data.hdop, 2, 1, hdop);
   dtostrf (data.gsa_data.vdop, 2, 1, vdop);

   snprintf(msg, MSG_SZ, "PDOP:\t\t\t[ %s ]\r\n",
            pdop);
   Serial.print(msg);

   snprintf(msg, MSG_SZ, "HDOP:\t\t\t[ %s ]\r\n",
            hdop);
   Serial.print(msg);

   snprintf(msg, MSG_SZ, "VDOP:\t\t\t[ %s ]\r\n",
            vdop);
   Serial.print(msg);

   Serial.print("\r\n\n>");

   return;
}

void getGSVInfo()
{
   data = gps->getData();
   int32_t i;
   int32_t tot_sats = data.gsv_data.tot_sats;
   int32_t current_sats = data.gsv_data.current_sats;
   int32_t amount = data.gsv_data.amount;
   int32_t number = data.gsv_data.number;

   uint8_t degree_ext_ASCII_char = 248;

   Serial.print("\r\n");

   snprintf(msg, MSG_SZ,  "Constellation:\t\t[ %s ]\t",
            data.gsv_data.constellation);
   Serial.print(msg);

   if (strcmp((char*)data.gsv_data.constellation, "$GPGSV") == 0)
   {
      Serial.print("-- message to report all GPS satellites\r\n");
   }
   else if (strcmp((char*)data.gsv_data.constellation, "$GLGSV") == 0)
   {
      Serial.print("-- message to report all GLONASS satellites\r\n");
   }
   else if (strcmp((char*)data.gsv_data.constellation, "$GAGSV") == 0)
   {
      Serial.print("-- message to report all GALILEO satellites\r\n");
   }
   else if (strcmp((char*)data.gsv_data.constellation, "$BDGSV") == 0)
   {
      Serial.print("-- message to report all BEIDOU satellites\r\n");
   }
   else if (strcmp((char*)data.gsv_data.constellation, "$QZGSV") == 0)
   {
      Serial.print("-- message to report all QZSS satellites\r\n");
   }
   else if (strcmp((char*)data.gsv_data.constellation, "$GNGSV") == 0)
   {
      Serial.print("-- message to report all satellites for all enabled constellations\r\n");
   }
   else
   {
      /* nothing to do */
   }

   snprintf(msg, MSG_SZ, "GSV message:\t\t[ %ld of %ld ]\r\n", number, amount);
   Serial.print(msg);

   snprintf(msg, MSG_SZ, "Num of Satellites:\t[ %ld of %ld ]\r\n", data.gsv_data.current_sats, tot_sats);
   Serial.print(msg);

   Serial.print("\r\n");

   for (i=0; i<current_sats; i++)
   {
      snprintf(msg, MSG_SZ, "Sat%02ldPRN:\t\t[ %03ld ]\r\n", i+1+((number-1)*GSV_MSG_SATS),
               data.gsv_data.gsv_sat_i[i].prn);
      Serial.print(msg);

      snprintf(msg, MSG_SZ, "Sat%02ldElev (%c):\t\t[ %03ld ]\r\n", i+1+((number-1)*GSV_MSG_SATS), degree_ext_ASCII_char,
               data.gsv_data.gsv_sat_i[i].elev);
      Serial.print(msg);

      snprintf(msg, MSG_SZ, "Sat%02ldAzim (%c):\t\t[ %03ld ]\r\n", i+1+((number-1)*GSV_MSG_SATS), degree_ext_ASCII_char,
               data.gsv_data.gsv_sat_i[i].azim);
      Serial.print(msg);

      snprintf(msg, MSG_SZ, "Sat%02ldCN0 (dB):\t\t[ %03ld ]\r\n", i+1+((number-1)*GSV_MSG_SATS),
               data.gsv_data.gsv_sat_i[i].cn0);
      Serial.print(msg);

      Serial.print("\r\n");
   }

   Serial.print("\r\n>");
}

void printHelp(void)
{
   Serial.print("\r\n 1) GETPOS:\r\n\tGets and decode the first useful $GPGGA NMEA string with the global position information.\n");
   Serial.print("\r\n 2) LASTPOS:\r\n\tPrints the last saved position from the GPS reception process.\n");
   Serial.print("\r\n 3) WAKESTATUS:\r\n\tGets the activity state of the GPS datas reception.\r\n\tWill be printed one of the two states: 0 / 1.\n");
   Serial.print("\r\n 4) HELP:\r\n\tPrints command menu.\n");
   Serial.print("\r\n 5) DEBUG:\r\n\tChanges the debug state of the application (default is ON).\r\n\tIf debug is ON, when the getpos command is sent, the just decoded position will be printed.\n");
   Serial.print("\r\n 6) TRACK:\r\n\tBegins the tracking position process.\r\n\tYou have to choose the number of positions that you want to track and the delay between two\r\n\treceptions.\n");
   Serial.print("\r\n 7) LASTTRACK:\r\n\tIf last tracking process went good, prints last tracked positions on the console.\n");
   Serial.print("\r\n 8) GETFWVER:\r\n\tSends the ST proprietary $PSTMGETSWVER NMEA command (to be written on serial terminal) and decode the answer with all info about the FW version.\n");
   Serial.print("\r\n 9) GETGNSMSG:\r\n\tGets and decode the first useful NMEA string (the $--GNS one) with fix data for single or combined satellite navigation system information.\n");
   Serial.print("\r\n10) GETGPGST:\r\n\tGets and decode the first useful $GPGST NMEA string with the GPS Pseudorange Noise Statistics.\n");
   Serial.print("\r\n11) GETGPRMC:\r\n\tGets and decode the first useful $GPRMC NMEA string with the Recommended Minimum Specific GPS/Transit data.\n");
   Serial.print("\r\n12) GETGSAMSG:\r\n\tGets and decode the first useful NMEA string (the $--GSA one) with GNSS DOP and active satellites information.\n");
   Serial.print("\r\n13) GETGSVMSG:\r\n\tGets and decode the first useful NMEA string (the $--GSV one) with GNSS Satellites in View information.\r\n\n");
   Serial.print("\r\n19) EXT-HELP:\r\n\tPrints this extended help.\n");
   Serial.print("\r\n> ");
}

int trackGotPos(uint32_t how_many, uint32_t time)
{
   int tracked = 0;
   uint32_t i = 0;
   int startTime = millis();
   while (i<how_many)
   {
      gps->update();
      if (millis() >= (startTime + (time*1000)))
      {
         data = gps->getData();
         if(data.gpgga_data.valid != (uint8_t)VALID)
         {
            break;
         }
         tracked++;
         snprintf(msg, MSG_SZ,  "Position %ld just get.\r\n", i + 1U);
         Serial.print(msg);
         if(data.debug == DEBUG_ON)
            printValidInfo();
         stored_positions[i] = data.gpgga_data;
         i++;
         startTime = millis();
      }
   }
   return tracked;
}

void printTrackedPositions (uint32_t how_many)
{
   for(uint32_t i = 0; i < how_many; i++)
   {
      int lat = (int) (stored_positions[i].xyz.lat/100.0);
      int lat_mod = (int) (stored_positions[i].xyz.lat) - (lat * 100);
      int lon = (int) (stored_positions[i].xyz.lon/100.0);
      int lon_mod = (int) (stored_positions[i].xyz.lon) - (lon * 100);
      char alt[7];
      char acc[5];
      dtostrf (stored_positions[i].xyz.alt, 3, 2, alt);
      dtostrf (stored_positions[i].acc, 4, 1, acc);

      snprintf(msg, MSG_SZ,  "Position n. %ld:\r\n", i + 1U);
      Serial.print(msg);

      snprintf(msg, MSG_SZ,  "UTC:\t\t\t[ %02ld:%02ld:%02ld ]\r\n",
               stored_positions[i].utc.hh, stored_positions[i].utc.mm, stored_positions[i].utc.ss);
      Serial.print(msg);

      snprintf(msg, MSG_SZ,  "Latitude:\t\t[ %02d' %02d'' %c ]\r\n",
               lat,
               lat_mod,
               stored_positions[i].xyz.ns);
      Serial.print(msg);

      snprintf(msg, MSG_SZ,  "Longitude:\t\t[ %02d' %02d'' %c ]\r\n",
               lon,
               lon_mod,
               stored_positions[i].xyz.ew);
      Serial.print(msg);

      snprintf(msg, MSG_SZ,  "Satellites locked:\t[ %ld ]\r\n",
               stored_positions[i].sats);
      Serial.print(msg);

      snprintf(msg, MSG_SZ,  "Position accuracy:\t[ %s ]\r\n",
               acc);
      Serial.print(msg);

      snprintf(msg, MSG_SZ,  "Altitude:\t\t[ %s%c ]\r\n",
               alt,
               (stored_positions[i].xyz.mis + 32U));
      Serial.print(msg);

      snprintf(msg, MSG_SZ,  "Geoid infos:\t\t[ %ld%c ]\r\n",
               stored_positions[i].geoid.height,
               stored_positions[i].geoid.mis);
      Serial.print(msg);

      snprintf(msg, MSG_SZ,  "Diff update:\t\t[ %ld ]\r\n",
               stored_positions[i].update);
      Serial.print(msg);

      Serial.print("\r\n\n>");
   }
}


void getPSTMVERInfo ()
{
   data = gps->getData();
   if(strlen((char *)data.pstmver_data.pstmver_string) != 0U )
   {
      Serial.print("\r\n");
      snprintf(msg, MSG_SZ,  "Version Info:\t\t[ %s ]\t",
               data.pstmver_data.pstmver_string);
      Serial.print(msg);

      Serial.print("\r\n\n");
   }
   else
   {
      snprintf(msg, MSG_SZ,  "No version info available.\r\n\n");
      Serial.print(msg);
   }

   Serial.print("\r\n>");
}
