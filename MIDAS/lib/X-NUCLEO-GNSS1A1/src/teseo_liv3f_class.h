/**
  *******************************************************************************
  * @file    teseo_liv3f_class.h
  * @author  AST
  * @version V1.0.0
  * @date    Jan-2019
  *
  *******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        www.st.com/software_license_agreement_liberty_v2
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
  ********************************************************************************
  */
#include "Arduino.h"
#include "Wire.h"
#include "teseo.h"
#include "gnss_parser.h"
#include "NMEA_parser.h"

#define DEFAULT_BUS 0
#define DEFAULT_I2C NULL
#define DEFAULT_UART NULL

#define DEFAULT_DEVICE_ADDRESS 0x3A
#define DEFAULT_DEVICE_PORT 0xFF

#define BUFFER_SIZE 32
#define MAX_FIELD_LENGTH 30
#define MAX_STRING_LENGTH 100
#define MAX_RESPONSE_LENGTH 40


typedef struct
{
   char inputString[MAX_STRING_LENGTH];
   char inputString2[MAX_STRING_LENGTH];
   bool stringComplete;
   char inChar[BUFFER_SIZE];
   int index;
   int end;
} I2CHandler;


typedef struct
{
   char inputString[MAX_STRING_LENGTH];
   bool stringComplete;
   int index;
   int end;
} UARTHandler;

typedef enum
{
   TENS = 0,
   HUNDREDS,
   THOUSANDS
} Decimal_t;


/** Class representing a Teseo-LIV3F component
 */
class TeseoLIV3F
{

public:

   TeseoLIV3F(TwoWire *i2c, int resetPin, int enablePin) : dev_i2c(i2c), pinRes(resetPin), pinEn(enablePin)
   {
      pinMode(pinRes, OUTPUT);
      pinMode(pinEn, OUTPUT);
      useI2C = 1;
      i2ch.stringComplete = false;
      i2ch.index = 0;
      i2ch.end = 0;
      commandDone = 1;
   }

   TeseoLIV3F(HardwareSerial *uart, int resetPin, int enablePin) : dev_uart(uart), pinRes(resetPin), pinEn(enablePin)
   {
      pinMode(pinRes, OUTPUT);
      pinMode(pinEn, OUTPUT);
      uarth.stringComplete = false;
      uarth.index = 0;
      uarth.end = 0;
      commandDone = 1;
   }

   /**
    * @brief       Initialize the sensor and the data structures
    * @note		in case of I2C communication, the TwoWire @a begin() should always be called after this function
    * @return      GNSS_OK on Success
    */
   GNSS_StatusTypeDef init()
   {
      digitalWrite(pinRes, LOW);
      delay(1000);
      digitalWrite(pinRes, HIGH);
      delay(5000);
      GNSS_PARSER_Init(&data);
      if (useI2C)
      {
         memset(i2ch.inputString, 0, MAX_STRING_LENGTH);
         memset(i2ch.inputString2, 0, MAX_STRING_LENGTH);
		 //Reinitialize the i2c communication
         //Note: it is critical for this method to be called AFTER the device initialization
#ifdef ARDUINO_ARCH_STM32
         Wire.end();
#endif
         Wire.begin();
      }
      sendCommand((char *)"$PSTMRESTOREPAR");
      sendCommand((char *)"$PSTMSRR");
      delay(4000);
      return GNSS_OK;
   }

   /**
    * @brief       Update the internal data structures of the sensor using the appropriate communication method
    * @note		To prevent data loss, this function should be called at least 20 times per second
    * @return      GNSS_OK on Success
    */
   GNSS_StatusTypeDef update()
   {
      if (useI2C)
         return I2CUpdate();
      else
         return UARTUpdate();
   }

   /**
    * @brief       	Send a command to the device
    * @param command	The command to send
    * @return      	GNSS_OK on Success
    */
   GNSS_StatusTypeDef sendCommand(char *command)
   {
      if (useI2C)
         WrWord(command);
      else
      {
         dev_uart->print(command);
         dev_uart->print("\r\n");
      }
      return GNSS_OK;
   }

   /**
    * @brief    	    Ask the device for a specific message
    * @param message	The message to recieve
    * @return   	    GNSS_OK on Success
    */
   GNSS_StatusTypeDef askMessage(char* message)
   {
      memset(compareMessage, 0, MAX_RESPONSE_LENGTH);
      strncpy(compareMessage, message, strlen(message));
      commandDone=0;
      return GNSS_OK;
   }

   /**
    * @brief       Ask the device if the message requested by @a askMessage() was recieved
    * @return      1 if the message was recieved, 0 otherwise
    */
   int getMessageDone ()
   {
      return commandDone;
   }

   /**
    * @brief       Get the complete data structure
    * @return      The full data structure
    */
   GNSSParser_Data_t getData()
   {
      return data;
   }

   /**
    * @brief       Get the wakeup status of the device
    * @return      1 if the device is enabled, 0 otherwise
    */
   int getWakeupStatus()
   {
      return digitalRead(pinEn);
   }

   /**
    * @brief       Get the GPGGA coordinates
    * @return      The coordinates structure
    */
   Coords_t getCoords()
   {
      return data.gpgga_data.xyz;
   }

   /**
    * @brief       Get the debug status of the device
    * @return      DEBUG_ON if debug is enabled, DEBUG_OFF otherwise
    */
   Debug_State getDebugStatus ()
   {
      return data.debug;
   }

   /**
    * @brief       Get the GPGGA message data structure
    * @return      The required data structure
    */
   GPGGA_Info_t getGPGGAData()
   {
      return data.gpgga_data;
   }

   /**
    * @brief       Get the --GNS message data structure
    * @return      The required data structure
    */
   GNS_Info_t getGNSData ()
   {
      return data.gns_data;
   }

   /**
    * @brief       Get the GPGTS message data structure
    * @return      The required data structure
    */
   GPGST_Info_t getGPGSTData()
   {
      return data.gpgst_data;
   }

   /**
    * @brief       Get the GPRMC message data structure
    * @return      The required data structure
    */
   GPRMC_Info_t getGPRMCData ()
   {
      return data.gprmc_data;
   }

   /**
    * @brief       Get the --GSA message data structure
    * @return      The required data structure
    */
   GSA_Info_t getGSAData ()
   {
      return data.gsa_data;
   }

   /**
    * @brief       Get the --GSV message data structure
    * @return      The required data structure
    */
   GSV_Info_t getGSVData ()
   {
      return data.gsv_data;
   }

   /**
    * @brief       Get the PSTMVER message data structure
    * @return      The required data structure
    */
   PSTMVER_Info_t getVERData ()
   {
      return data.pstmver_data;
   }

   /**
    * @brief       Get the PSTMPASSRTN message data structure
    * @return      The required data structure
    */
   PSTMPASSRTN_Info_t getPASSData ()
   {
      return data.pstmpass_data;
   }

   /**
    * @brief       Get the PSTMAGPS message data structure
    * @return      The required data structure
    */
   PSTMAGPS_Info_t getAGPSData ()
   {
      return data.pstmagps_data;
   }

   /**
    * @brief       Get the Geofence message data structure
    * @return      The required data structure
    */
   Geofence_Info_t getGeofenceData ()
   {
      return data.geofence_data;
   }

   /**
    * @brief       Get the Odometer message data structure
    * @return      The required data structure
    */
   Odometer_Info_t getOdometerData ()
   {
      return data.odo_data;
   }

   /**
    * @brief       Get the Datalog structure
    * @return      The required data structure
    */
   Datalog_Info_t getDatalogData ()
   {
      return data.datalog_data;
   }

   /**
    * @brief       Get the result of the last command sent
    * @return      GNSS_OP_OK if it was a success, GNSS_OP_ERROR otherwise
    */
   OpResult_t getResult ()
   {
      return data.result;
   }

   /**
    * @brief       Activate/deactivate the debug flag
    * @return      The current state of the debug flag
    */
   Debug_State toggleDebug()
   {
      data.debug = (data.debug == DEBUG_ON ? DEBUG_OFF : DEBUG_ON);
      return data.debug;
   }

protected:

   /**
    * @brief       Update the internal data structures of the sensor using I2C communication
    * @return      GNSS_OK on Success
    */
   GNSS_StatusTypeDef I2CUpdate();
   /**
    * @brief       Update the internal data structures of the sensor using UART communication
    * @return      GNSS_OK on Success
    */
   GNSS_StatusTypeDef UARTUpdate();

   /**
    * @brief       	Sends the string to the I2C device
    * @param strToWr	The string to write
    * @return      	GNSS_OK on Success
    */
   GNSS_StatusTypeDef WrWord(char *strToWr);
   /**
    * @brief       Recieves 32 bytes from the I2C device
    * @param data	The buffer used to memorize the incoming bytes
    * @return      GNSS_OK on Success
    * @return		data pointer contains the recieved data
    */
   GNSS_StatusTypeDef RdWord(char *data);

   GNSS_StatusTypeDef I2CRead(uint16_t RegisterAddr, uint8_t* pBuffer, uint16_t NumByteToRead);
   GNSS_StatusTypeDef I2CWrite(uint16_t RegisterAddr, uint8_t* pBuffer, uint16_t NumByteToWrite);


   /**
    * @brief  This function initializes the agent handling parsed GNSS data
    * @param  pGNSSParser_Data The agent
    * @retval GNSS_PARSER_OK on success GNSS_PARSER_ERROR otherwise
    */
   GNSSParser_Status_t GNSS_PARSER_Init(GNSSParser_Data_t *pGNSSParser_Data);

   /**
    * @brief  This function computes the checksum and checks the sanity of a GNSS sentence
    * @param  pSentence The sentence
    * @param  len The sentence length
    * @retval GNSS_PARSER_OK on success GNSS_PARSER_ERROR otherwise
    */
   GNSSParser_Status_t GNSS_PARSER_CheckSanity(uint8_t *pSentence, uint64_t len);

   /**
    * @brief  This function dispatches a GNSS sentence to be parsed
    * @param  pGNSSParser_Data The agent
    * @param  msg The message type
    * @param  pBuffer The message to be dispatched
    * @retval GNSS_PARSER_OK on success GNSS_PARSER_ERROR otherwise
    */
   GNSSParser_Status_t GNSS_PARSER_ParseMsg(GNSSParser_Data_t *pGNSSParser_Data, uint8_t msg, uint8_t *pBuffer);

   /**
    * @brief  Function that makes the parsing of the $GPGGA NMEA string with all Global Positioning System Fixed data.
    * @param  pGPGGAInfo     Pointer to GPGGA_Info_t struct
    * @param  NMEA	          NMEA string read by the Gps expansion
    * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
    */
   ParseStatus_t NMEA_ParseGPGGA(GPGGA_Info_t *pGPGGAInfo, uint8_t NMEA[]);

   void scan_utc(uint8_t *pUTCStr, UTC_Info_t *pUTC);

   uint32_t nmea_checksum(const uint8_t buf[]);

   /**
    * @brief  Function that makes the parsing of the string read by the Gps expansion, capturing the right parameters from it.
    * @param  pGNSInfo      Pointer to GNS_Info_t struct
    * @param  NMEA[]        NMEA string read by the Gps expansion
    * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
    */
   ParseStatus_t NMEA_ParseGNS(GNS_Info_t *pGNSInfo, uint8_t NMEA[]);

   int32_t NMEA_CheckGNSMsg(const char header[]);

   /**
    * @brief  Function that makes the parsing of the $GPGST NMEA string with GPS Pseudorange Noise Statistics.
    * @param  pGPGSTInfo    Pointer to a GPGST_Info_t struct
    * @param  NMEA	         NMEA string read by the Gps expansion.
    * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
    */
   ParseStatus_t NMEA_ParseGPGST(GPGST_Info_t *pGPGSTInfo, uint8_t NMEA[]);

   /**
    * @brief  Function that makes the parsing of the $GPRMC NMEA string with Recommended Minimum Specific GPS/Transit data.
    * @param  pGPRMCInfo    Pointer to a GPRMC_Info_t struct
    * @param  NMEA	         NMEA string read by the Gps expansion.
    * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
    */
   ParseStatus_t NMEA_ParseGPRMC(GPRMC_Info_t *pGPRMCInfo, uint8_t NMEA[]);

   /**
    * @brief  Function that makes the parsing of the $GSA NMEA string.
    * @param  pGSAInfo      Pointer to a GSA_Info_t struct
    * @param  NMEA	         NMEA string read by the Gps expansion.
    * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
    */
   ParseStatus_t NMEA_ParseGSA(GSA_Info_t *pGSAInfo, uint8_t NMEA[]);

   int32_t NMEA_CheckGSAMsg(const char header[]);

   /**
    * @brief  Function that makes the parsing of the $GSV NMEA string.
    * @param  pGSVInfo      Pointer to a GSV_Info_t struct
    * @param  NMEA	         NMEA string read by the Gps expansion.
    * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
    */
   ParseStatus_t NMEA_ParseGSV(GSV_Info_t *pGSVInfo, uint8_t NMEA[]);

   int32_t NMEA_CheckGSVMsg(const char header[]);

   void NMEA_ResetGSVMsg(GSV_Info_t *pGSVInfo);

   /**
    * @brief  Function that parses of the $PSTMVER NMEA string with version data.
    * @param  pPSTMVER      Pointer to PSTMVER_Info_t struct
    * @param  NMEA	         NMEA string read by the Gps expansion
    * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
    */
   ParseStatus_t NMEA_ParsePSTMVER(PSTMVER_Info_t *pPSTMVER, uint8_t NMEA[]);

   /**
    * @brief  This function parses the geofence related messages
    * @param  pGeofence     Pointer to Geofence_Info_t
    * @param  NMEA	         NMEA string read by the Gps expansion.
    * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
    */
   ParseStatus_t NMEA_ParsePSTMGeofence(Geofence_Info_t *pGeofence, uint8_t NMEA[]);

   int32_t NMEA_CheckGeofenceMsg(const char header[]);

   void scan_timestamp_time(uint8_t buf[], Timestamp_Info_t *pTimestamp);

   void scan_timestamp_date(uint8_t buf[], Timestamp_Info_t *pTimestamp);

   uint32_t digit2int(uint8_t buf[], int32_t offset, Decimal_t d);

   /**
    * @brief  This function parses the odometer related messages
    * @param  pOdo          Pointer to a Odometer_Info_t struct
    * @param  NMEA          NMEA string read by the Gps expansion.
    * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
    */
   ParseStatus_t NMEA_ParsePSTMOdo(Odometer_Info_t *pOdo, uint8_t NMEA[]);

   int32_t NMEA_CheckOdoMsg(const char header[]);

   /**
    * @brief  This function parses the datalog related messages
    * @param  pDatalog      Pointer to a Datalog_Info_t struct
    * @param  NMEA          NMEA string read by the Gps expansion.
    * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
    */
   ParseStatus_t NMEA_ParsePSTMDatalog(Datalog_Info_t *pDatalog, uint8_t NMEA[]);

   int32_t NMEA_CheckDatalogMsg(const char header[]);

   /**
    * @brief  This function parses the list configuration message
    * @param  pResult             Ack from Teseo
    * @param  NMEA                NMEA string read by the Gps expansion.
    * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
    */
   ParseStatus_t NMEA_ParsePSTMsgl(OpResult_t *pResult, uint8_t NMEA[]);

   int32_t NMEA_CheckListMsg(const char header[]);

   /**
    * @brief  This function parses the SavePar messages
    * @param  pResult             Ack from Teseo
    * @param  NMEA                NMEA string read by the Gps expansion.
    * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
    */
   ParseStatus_t NMEA_ParsePSTMSavePar(OpResult_t *pResult, uint8_t NMEA[]);

   int32_t NMEA_CheckSaveparMsg(const char header[]);


   /**
    * @brief  Function that parses of the $PSTMSTAGPSPASSRTN NMEA string with version data.
    * @param  pPSTMPASSRTN  Pointer to PSTMPASSRTN_Info_t struct
    * @param  NMEA	         NMEA string read by the Gps expansion
    * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
    */
   ParseStatus_t NMEA_ParsePSTMPASSRTN(PSTMPASSRTN_Info_t *pPSTMPASSRTN, uint8_t NMEA[]);

   int32_t NMEA_CheckPassMsg(const char header[]);

   /**
    * @brief  Function that parses of the $PSTMSTAGPS NMEA string with version data.
    * @param  pPSTMAGPS Pointer to PSTMAGPS_Info_t struct
    * @param  NMEA	     NMEA string read by the Gps expansion
    * @retval PARSE_SUCC if the parsing process goes ok, PARSE_FAIL if it doesn't
    */
   ParseStatus_t NMEA_ParsePSTMAGPS(PSTMAGPS_Info_t *pPSTMAGPS, uint8_t NMEA[]);

   int32_t NMEA_CheckAGPSMsg(const char header[]);

   /**
    * @brief  This function makes a copy of the datas stored into GPGGAInfo into the pInfo param
    * @param  pInfo     Pointer to GPGGA_Info_t object where there are the GPGGA_Info_t to be copied
    * @param  GPGGAInfo Instance of a GPGGA_Info_t object pointer where the GPGGA_Info_t stored into pInfo have to be copied
    * @retval None
    */
   void NMEA_Copy_Data(GPGGA_Info_t *pInfo, GPGGA_Info_t GPGGAInfo);

   /**
    * @brief  This function converts a character to unsigned integer
    * @param  c The character to convert
    * @retval The returned unsigned integer
    */
   uint32_t char2int(uint8_t c);




   int useI2C = DEFAULT_BUS;
   TwoWire *dev_i2c = DEFAULT_I2C;
   HardwareSerial *dev_uart = DEFAULT_UART;
   int pinRes;
   int pinEn;
   int commandDone;
   char compareMessage[MAX_RESPONSE_LENGTH];
   I2CHandler i2ch;
   UARTHandler uarth;
   GNSSParser_Data_t data;
   uint8_t app[MAX_MSG_LEN][MAX_FIELD_LENGTH];
};
