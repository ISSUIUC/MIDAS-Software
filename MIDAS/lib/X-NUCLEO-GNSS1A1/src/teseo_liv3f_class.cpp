/**
  *******************************************************************************
  * @file    teseo_liv3f_class.cpp
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

#include <stdlib.h>
#include "Arduino.h"
#include "teseo_liv3f_class.h"




/*
 * Constant for strtol base param
 */
#define BASE 10
#define strtof(A, B) strtod(A, B)


#define MAX_TRANSFER_SIZE 200

GNSS_StatusTypeDef TeseoLIV3F::I2CUpdate()
{

   memset (i2ch.inChar, 0, BUFFER_SIZE);
   RdWord(i2ch.inChar);
   for (int i=0; i<BUFFER_SIZE; i++)
   {
      if (i2ch.inChar[i] != (char) 0x00)
      {
         if (i2ch.stringComplete)
            i2ch.inputString2[i2ch.index] = i2ch.inChar[i];
         else
            i2ch.inputString[i2ch.index] = i2ch.inChar[i];
         i2ch.index = i2ch.index + 1;
      }
      if (i2ch.inChar[i] == '\n')
      {
         i2ch.stringComplete = true;
         i2ch.end = i2ch.index;
         i2ch.index = 0;
      }
   }

   delay(1);

   if (i2ch.stringComplete)
   {
      uint8_t buffer[MAX_STRING_LENGTH];
      memset (buffer, 0, MAX_STRING_LENGTH);
      for (int i = 0; i < MAX_STRING_LENGTH; i++)
         buffer[i] = (uint8_t) i2ch.inputString[i];
      GNSSParser_Status_t status = GNSS_PARSER_CheckSanity(buffer, strlen((char *) buffer));
      if (status != GNSS_PARSER_ERROR)
      {
         if ((commandDone == 0) && (strncmp(compareMessage, (char *) buffer, strlen(compareMessage)) == 0))
         {
            commandDone = 1;
         }
         for(int m = 0; m < NMEA_MSGS_NUM; m++)
         {
            GNSS_PARSER_ParseMsg(&data, (eNMEAMsg)m, buffer);
         }
      }
      strncpy(i2ch.inputString, i2ch.inputString2, sizeof(i2ch.inputString));
      memset(i2ch.inputString2, 0, sizeof(i2ch.inputString2));
      i2ch.stringComplete = false;
   }
   return GNSS_OK;
}


GNSS_StatusTypeDef TeseoLIV3F::UARTUpdate()
{
   if (uarth.stringComplete)
   {
      uint8_t buffer[MAX_STRING_LENGTH];
      memset(buffer, 0, MAX_STRING_LENGTH);
      for (int i = 0; i < MAX_STRING_LENGTH; i++)
         buffer[i] = (uint8_t) uarth.inputString[i];
      GNSSParser_Status_t status = GNSS_PARSER_CheckSanity(buffer, uarth.end);
      if (status != GNSS_PARSER_ERROR)
      {
         if ((commandDone == 0) && (strncmp(compareMessage, (char *) buffer, strlen(compareMessage)) == 0))
         {
            commandDone = 1;
         }
         for(int m = 0; m < NMEA_MSGS_NUM; m++)
         {
            GNSS_PARSER_ParseMsg(&data, (eNMEAMsg)m, buffer);
         }
      }
      memset(uarth.inputString, 0, sizeof(uarth.inputString));
      uarth.stringComplete = false;
   }
   while (dev_uart->available())
   {
      char inputChar = dev_uart->read();
      uarth.inputString[uarth.index]= inputChar;
      if (inputChar == '\n')
      {
         uarth.stringComplete = true;
         uarth.end = uarth.index+1;
         uarth.index = 0;
         break;
      }
      uarth.index = uarth.index + 1;
   }
   return GNSS_OK;
}

GNSS_StatusTypeDef TeseoLIV3F::WrWord(char *strToWr)
{
   GNSS_StatusTypeDef status;
   uint8_t buffer[MAX_TRANSFER_SIZE];
   memset (buffer, 0, MAX_TRANSFER_SIZE);
   strncpy((char *)buffer, strToWr, strlen(strToWr));
   buffer[strlen(strToWr)] = (uint8_t) '\r';
   buffer[strlen(strToWr)+1] = (uint8_t) '\n';
   buffer[strlen(strToWr)+2] = (uint8_t) '\0';
   status = I2CWrite(DEFAULT_DEVICE_PORT, buffer, strlen((char *)buffer));
   return status;
}

GNSS_StatusTypeDef TeseoLIV3F::RdWord(char *data)
{
   GNSS_StatusTypeDef status;
   uint8_t buffer[MAX_TRANSFER_SIZE];
   memset(buffer, 0, MAX_TRANSFER_SIZE);
   status = I2CRead(DEFAULT_DEVICE_PORT, buffer, BUFFER_SIZE-1);
   if(!status)
   {
      for (int i=0; i<BUFFER_SIZE-1; i++)
      {
         data[i]= (buffer[i] == 0xFF) ? (char)0x00 : (char) buffer[i];
      }
   }
   return status;
}

GNSS_StatusTypeDef TeseoLIV3F::I2CRead(uint16_t RegisterAddr, uint8_t* pBuffer, uint16_t NumByteToRead)
{
   int status = 0;
//Loop until the port is transmitted correctly
   do
   {
#ifdef DEBUG_MODE
      Serial.print("Beginning transmission to ");
      Serial.println(((DEFAULT_DEVICE_ADDRESS)) & 0x7F);
#endif
      dev_i2c->beginTransmission(DEFAULT_DEVICE_ADDRESS);
#ifdef DEBUG_MODE
      Serial.print("Writing port number ");
      Serial.println(RegisterAddr);
#endif
      dev_i2c->write((uint8_t)RegisterAddr);
      status = dev_i2c->endTransmission(false);
//Fix for some STM32 boards
//Reinitialize th i2c bus with the default parameters

#ifdef ARDUINO_ARCH_STM32
      if (status)
      {
         dev_i2c->end();
         dev_i2c->begin();
      }
#endif
//End of fix
   }
   while(status != 0);

   dev_i2c->requestFrom((uint8_t)DEFAULT_DEVICE_ADDRESS, (uint8_t) NumByteToRead);

   int i=0;
   while (dev_i2c->available())
   {
      pBuffer[i] = dev_i2c->read();
      i++;
   }


   return (GNSS_StatusTypeDef) status;
}

GNSS_StatusTypeDef TeseoLIV3F::I2CWrite(uint16_t RegisterAddr, uint8_t* pBuffer, uint16_t NumByteToWrite)
{
   int status = 0;
//Loop until the port is transmitted correctly
   dev_i2c->beginTransmission(DEFAULT_DEVICE_ADDRESS);
   status += dev_i2c->write((uint8_t)RegisterAddr);
   int i=0;
   while (i<NumByteToWrite)
   {
      status += dev_i2c->write(pBuffer[i]);
      i++;
   }
#ifdef DEBUG_MODE
   if (status != (NumByteToWrite + 1))
      Serial.println("Not all bytes were transmitted");
#endif

   status = dev_i2c->endTransmission(true);

#ifdef DEBUG_MODE
   if (status != 0)
      Serial.println("Something went terribly wrong");
#endif


   return (GNSS_StatusTypeDef) 0;
}


GNSSParser_Status_t TeseoLIV3F::GNSS_PARSER_Init(GNSSParser_Data_t *pGNSSParser_Data)
{
   if (pGNSSParser_Data == NULL)
   {
      return GNSS_PARSER_ERROR;
   }

   pGNSSParser_Data->debug = DEBUG_ON;
   (void)memset((void *)(&pGNSSParser_Data->gpgga_data), 0, sizeof(GPGGA_Info_t));
   pGNSSParser_Data->gpgga_data.xyz.ew = (uint8_t)' ';
   pGNSSParser_Data->gpgga_data.xyz.ns = (uint8_t)' ';
   pGNSSParser_Data->gpgga_data.xyz.mis = (uint8_t)' ';

   (void)memset((void *)(&pGNSSParser_Data->gns_data), 0, sizeof(GNS_Info_t));
   pGNSSParser_Data->gns_data.xyz.ew = (uint8_t)' ';
   pGNSSParser_Data->gns_data.xyz.ns = (uint8_t)' ';

   (void)memset((void *)(&pGNSSParser_Data->gpgst_data), 0, sizeof(GPGST_Info_t));

   (void)memset((void *)(&pGNSSParser_Data->gprmc_data), 0, sizeof(GPRMC_Info_t));
   pGNSSParser_Data->gprmc_data.xyz.ew = (uint8_t)' ';
   pGNSSParser_Data->gprmc_data.xyz.ns = (uint8_t)' ';

   (void)memset((void *)(&pGNSSParser_Data->gsa_data), 0, sizeof(GSA_Info_t));
   (void)memset((void *)(&pGNSSParser_Data->gsv_data), 0, sizeof(GSV_Info_t));
   (void)memset((void *)(&pGNSSParser_Data->pstmver_data), 0, sizeof(PSTMVER_Info_t));
   (void)memset((void *)(&pGNSSParser_Data->pstmpass_data), 0, sizeof(PSTMPASSRTN_Info_t));
   (void)memset((void *)(&pGNSSParser_Data->pstmagps_data), 0, sizeof(PSTMAGPS_Info_t));
   (void)memset((void *)(&pGNSSParser_Data->geofence_data), 0, sizeof(Geofence_Info_t));
   (void)memset((void *)(&pGNSSParser_Data->odo_data), 0, sizeof(Odometer_Info_t));
   (void)memset((void *)(&pGNSSParser_Data->datalog_data), 0, sizeof(Datalog_Info_t));
   (void)memset((void *)(&pGNSSParser_Data->result), 0, sizeof(OpResult_t));

   return GNSS_PARSER_OK;
}

GNSSParser_Status_t TeseoLIV3F::GNSS_PARSER_CheckSanity(uint8_t *pSentence, uint64_t len)
{
   uint32_t checksum, check = 0U;

   if((len > 0U) && (len < 5U))
   {
      return GNSS_PARSER_ERROR;
   }
   if(len == 0U)
   {
      return GNSS_PARSER_OK;
   }
   checksum = (char2int(pSentence[len-4U]) << 4) | char2int(pSentence[len-3U]);

   for(uint64_t c = 1U; c < (len-5U); c++)
   {
      check = (check ^ pSentence[c]);
   }

   return (check == checksum) ? GNSS_PARSER_OK : GNSS_PARSER_ERROR;
}


GNSSParser_Status_t TeseoLIV3F::GNSS_PARSER_ParseMsg(GNSSParser_Data_t *pGNSSParser_Data, uint8_t msg, uint8_t *pBuffer)
{
   ParseStatus_t status = PARSE_FAIL;

   switch(msg)
   {
   case GPGGA:
      status = NMEA_ParseGPGGA(&pGNSSParser_Data->gpgga_data, pBuffer);
      break;
   case GNS:
      status = NMEA_ParseGNS(&pGNSSParser_Data->gns_data, pBuffer);
      break;
   case GPGST:
      status = NMEA_ParseGPGST(&pGNSSParser_Data->gpgst_data, pBuffer);
      break;
   case GPRMC:
      status = NMEA_ParseGPRMC(&pGNSSParser_Data->gprmc_data, pBuffer);
      break;
   case GSA:
      status = NMEA_ParseGSA(&pGNSSParser_Data->gsa_data, pBuffer);
      break;
   case GSV:
      status = NMEA_ParseGSV(&pGNSSParser_Data->gsv_data, pBuffer);
      break;
   case PSTMVER:
      status = NMEA_ParsePSTMVER(&pGNSSParser_Data->pstmver_data, pBuffer);
      break;
   case PSTMPASSRTN:
      status = NMEA_ParsePSTMPASSRTN(&pGNSSParser_Data->pstmpass_data, pBuffer);
      break;
   case PSTMAGPSSTATUS:
      status = NMEA_ParsePSTMAGPS(&pGNSSParser_Data->pstmagps_data, pBuffer);
      break;
   case PSTMGEOFENCE:
      status = NMEA_ParsePSTMGeofence(&pGNSSParser_Data->geofence_data, pBuffer);
      break;
   case PSTMODO:
      status = NMEA_ParsePSTMOdo(&pGNSSParser_Data->odo_data, pBuffer);
      break;
   case PSTMDATALOG:
      status = NMEA_ParsePSTMDatalog(&pGNSSParser_Data->datalog_data, pBuffer);
      break;
   case PSTMSGL:
      status = NMEA_ParsePSTMsgl(&pGNSSParser_Data->result, pBuffer);
      break;
   case PSTMSAVEPAR:
      status = NMEA_ParsePSTMSavePar(&pGNSSParser_Data->result, pBuffer);
      break;
   default:
      break;
   }

   return ((status == PARSE_FAIL) ? GNSS_PARSER_ERROR : GNSS_PARSER_OK);
}





ParseStatus_t TeseoLIV3F::NMEA_ParseGPGGA(GPGGA_Info_t *pGPGGAInfo, uint8_t NMEA[])
{
   int32_t new_field;
   BOOL valid_msg = FALSE;

   ParseStatus_t status = PARSE_FAIL;

   if(NMEA != NULL)
   {

      /* clear the app[][] buffer */
      for (int8_t i = 0; i < MAX_MSG_LEN; i++)
      {
         (void)memset(app[i], 0, (size_t)MAX_FIELD_LENGTH);
      }

      for(int32_t i = 0, j = 0, k = 0; (NMEA[i] != (uint8_t)'\n'); i++)
      {
         new_field = 0;

         if ((NMEA[i] == (uint8_t)',') || (NMEA[i] == (uint8_t)'*'))
         {
            app[j][k] = (uint8_t)'\0';
            new_field = 1;

            if (strcmp((char *)app[0], "$GPGGA") == 0)
            {
               j++;
               k = 0;
               valid_msg = TRUE;
            }
            else
            {
               break;
            }
         }
         if(new_field == 0)
         {
            app[j][k] = NMEA[i];
            k++;
         }
      }

      if (valid_msg == TRUE)
      {
         int32_t valid = strtol((char *)app[6], NULL, BASE);
         if((valid == 1) || (valid == 0))
         {
            pGPGGAInfo->valid = (uint8_t)valid;
         }

         scan_utc(app[1], &pGPGGAInfo->utc);
         pGPGGAInfo->xyz.lat = strtod((char *)app[2], NULL);
         pGPGGAInfo->xyz.ns = *((uint8_t*)app[3]);
         pGPGGAInfo->xyz.lon = strtod((char *)app[4], NULL);
         pGPGGAInfo->xyz.ew = *((uint8_t*)app[5]);
         pGPGGAInfo->sats = strtol((char *)app[7], NULL, BASE);
         pGPGGAInfo->acc = strtof((char *)app[8], NULL);
         pGPGGAInfo->xyz.alt = strtof((char *)app[9], NULL);
         pGPGGAInfo->xyz.mis = *((uint8_t*)app[10]);
         pGPGGAInfo->geoid.height = strtol((char *)app[11], NULL, BASE);
         pGPGGAInfo->geoid.mis = *((uint8_t*)app[12]);
         // This field is reserved
         //pGPGGAInfo->update = strtol((char *)app[13], NULL, BASE);
         pGPGGAInfo->checksum = nmea_checksum(app[15]);

         status = PARSE_SUCC;
      }
   }

   return status;
}

void TeseoLIV3F::scan_utc(uint8_t *pUTCStr, UTC_Info_t *pUTC)
{
   pUTC->utc = strtol((char *)pUTCStr,NULL,10);

   pUTC->hh = (pUTC->utc / 10000);
   pUTC->mm = (pUTC->utc - (pUTC->hh * 10000)) / 100;
   pUTC->ss = pUTC->utc - ((pUTC->hh * 10000) + (pUTC->mm * 100));

   return;
}

uint32_t TeseoLIV3F::nmea_checksum(const uint8_t buf[])
{
   return ((char2int(buf[0]) << 4) | (char2int(buf[1])));
}


ParseStatus_t TeseoLIV3F::NMEA_ParseGNS(GNS_Info_t *pGNSInfo, uint8_t NMEA[])
{
   int32_t new_field;
   BOOL valid_msg = FALSE;

   ParseStatus_t status = PARSE_FAIL;

   if(NMEA != NULL)
   {

      /* clear the app[][] buffer */
      for (int8_t i = 0; i < MAX_MSG_LEN; i++)
      {
         (void)memset(app[i], 0, (size_t)MAX_FIELD_LENGTH);
      }

      for (int32_t i = 0, j = 0, k = 0; (NMEA[i] != (uint8_t)'\n'); i++)
      {
         new_field = 0;

         if ((NMEA[i] == (uint8_t)',') || (NMEA[i] == (uint8_t)'*'))
         {
            app[j][k] = (uint8_t)'\0';
            new_field = 1;

            if (NMEA_CheckGNSMsg((char *)app[0]) == 0)
            {
               j++;
               k = 0;
               valid_msg = TRUE;
            }
            else
            {
               break;
            }
         }
         if(new_field == 0)
         {
            app[j][k] = NMEA[i];
            k++;
         }
      }

      if (valid_msg == TRUE)
      {
         (void)strncpy((char *)pGNSInfo->constellation, (char *)app[0], MAX_STR_LEN);
         scan_utc(app[1], &pGNSInfo->utc);
         pGNSInfo->xyz.lat = strtod((char *)app[2], NULL);
         pGNSInfo->xyz.ns = *((uint8_t*)app[3]);
         pGNSInfo->xyz.lon = strtod((char *)app[4], NULL);
         pGNSInfo->xyz.ew = *((uint8_t*)app[5]);
         pGNSInfo->gps_mode = *((uint8_t*)app[6]);
         pGNSInfo->glonass_mode = *((uint8_t*)app[7]);
         pGNSInfo->sats = strtol((char *)app[8], NULL, BASE);
         pGNSInfo->hdop = strtof((char *)app[9], NULL);
         pGNSInfo->xyz.alt = strtof((char *)app[10], NULL);
         pGNSInfo->geo_sep = strtof((char *)app[11], NULL);
         pGNSInfo->dgnss_age = *((uint8_t*)app[12]);
         pGNSInfo->dgnss_ref = *((uint8_t*)app[13]);
         pGNSInfo->checksum = nmea_checksum(app[14]);

         status = PARSE_SUCC;
      }
   }

   return status;
}

int32_t TeseoLIV3F::NMEA_CheckGNSMsg(const char header[])
{
   int32_t is_gnsmsg = 1;

   if (strcmp(header, "$GPGNS") == 0)
   {
      is_gnsmsg = 0;
   }
   if (strcmp(header, "$GAGNS") == 0)
   {
      is_gnsmsg = 0;
   }
   if (strcmp(header, "$BDGNS") == 0)
   {
      is_gnsmsg = 0;
   }
   if (strcmp(header, "$QZGNS") == 0)
   {
      is_gnsmsg = 0;
   }
   if (strcmp(header, "$GNGNS") == 0)
   {
      is_gnsmsg = 0;
   }

   return is_gnsmsg;
}

ParseStatus_t TeseoLIV3F::NMEA_ParseGPGST(GPGST_Info_t *pGPGSTInfo, uint8_t NMEA[])
{
   int32_t new_field;
   BOOL valid_msg = FALSE;

   ParseStatus_t status = PARSE_FAIL;

   if(NMEA != NULL)
   {

      /* clear the app[][] buffer */
      for (int8_t i = 0; i < MAX_MSG_LEN; i++)
      {
         (void)memset(app[i], 0, (size_t)MAX_FIELD_LENGTH);
      }

      for (int32_t i = 0, j = 0, k = 0; (NMEA[i] != (uint8_t)'\n'); i++)
      {
         new_field = 0;

         if ((NMEA[i] == (uint8_t)',') || (NMEA[i] == (uint8_t)'*'))
         {
            app[j][k] = (uint8_t)'\0';
            new_field = 1;

            if (strcmp((char *)app[0], "$GPGST") == 0)
            {
               j++;
               k = 0;
               valid_msg = TRUE;
            }
            else
            {
               break;
            }
         }
         if(new_field == 0)
         {
            app[j][k] = NMEA[i];
            k++;
         }
      }

      if (valid_msg == TRUE)
      {
         scan_utc(app[1], &pGPGSTInfo->utc);
         pGPGSTInfo->EHPE = strtof((char *)app[2], NULL);
         pGPGSTInfo->semi_major_dev = strtof((char *)app[3], NULL);
         pGPGSTInfo->semi_minor_dev = strtof((char *)app[4], NULL);
         pGPGSTInfo->semi_major_angle = strtof((char *)app[5], NULL);
         pGPGSTInfo->lat_err_dev = strtof((char *)app[6], NULL);
         pGPGSTInfo->lon_err_dev = strtof((char *)app[7], NULL);
         pGPGSTInfo->alt_err_dev = strtof((char *)app[8], NULL);
         pGPGSTInfo->checksum = nmea_checksum(app[9]);

         status = PARSE_SUCC;
      }
   }

   return status;
}


ParseStatus_t TeseoLIV3F::NMEA_ParseGPRMC(GPRMC_Info_t *pGPRMCInfo, uint8_t NMEA[])
{
   int32_t new_field;
   BOOL valid_msg = FALSE;

   ParseStatus_t status = PARSE_FAIL;

   if(NMEA != NULL)
   {

      /* clear the app[][] buffer */
      for (int8_t i = 0; i < MAX_MSG_LEN; i++)
      {
         (void)memset(app[i], 0, (size_t)MAX_FIELD_LENGTH);
      }

      for (int32_t i = 0, j = 0, k = 0; (NMEA[i] != (uint8_t)'\n'); i++)
      {
         new_field = 0;

         if ((NMEA[i] == (uint8_t)',') || (NMEA[i] == (uint8_t)'*'))
         {
            app[j][k] = (uint8_t)'\0';
            new_field = 1;

            if (strcmp((char *)app[0], "$GPRMC") == 0)
            {
               j++;
               k = 0;
               valid_msg = TRUE;
            }
            else
            {
               break;
            }
         }
         if(new_field == 0)
         {
            app[j][k] = NMEA[i];
            k++;
         }
      }

      if (valid_msg == TRUE)
      {
         scan_utc(app[1],  &pGPRMCInfo->utc);
         pGPRMCInfo->status = *((uint8_t*)app[2]);
         pGPRMCInfo->xyz.lat = strtod((char *)app[3], NULL);
         pGPRMCInfo->xyz.ns = *((uint8_t*)app[4]);
         pGPRMCInfo->xyz.lon = strtod((char *)app[5], NULL);
         pGPRMCInfo->xyz.ew = *((uint8_t*)app[6]);
         pGPRMCInfo->speed = strtof((char *)app[7], NULL);
         pGPRMCInfo->trackgood = strtof((char *)app[8], NULL);
         pGPRMCInfo->date = strtol((char *)app[9], NULL, BASE);
         pGPRMCInfo->mag_var = strtof((char *)app[10], NULL);
         pGPRMCInfo->mag_var_dir = *((uint8_t*)app[11]);
         /* WARNING: from received msg, it seems there is another data (app[12]) before the checksum */
         pGPRMCInfo->checksum = nmea_checksum(app[13]);

         status = PARSE_SUCC;
      }
   }

   return status;
}

ParseStatus_t TeseoLIV3F::NMEA_ParseGSA(GSA_Info_t *pGSAInfo, uint8_t NMEA[])
{
   int32_t new_field;
   BOOL valid_msg = FALSE;

   ParseStatus_t status = PARSE_FAIL;

   if(NMEA != NULL)
   {

      /* clear the app[][] buffer */
      for (int8_t i = 0; i < MAX_MSG_LEN; i++)
      {
         (void)memset(app[i], 0, (size_t)MAX_FIELD_LENGTH);
      }

      for (int32_t i = 0, j = 0, k = 0; (NMEA[i] != (uint8_t)(uint8_t)(uint8_t)(uint8_t)'\n'); i++)
      {
         new_field = 0;

         if ((NMEA[i] == (uint8_t)(uint8_t)(uint8_t)',') || (NMEA[i] == (uint8_t)(uint8_t)'*'))
         {
            app[j][k] = (uint8_t)'\0';
            new_field = 1;

            if (NMEA_CheckGSAMsg((char *)app[0]) == 0)
            {
               j++;
               k = 0;
               valid_msg = TRUE;
            }
            else
            {
               break;
            }
         }
         if(new_field == 0)
         {
            app[j][k] = NMEA[i];
            k++;
         }
      }

      if (valid_msg == TRUE)
      {
         (void)strncpy((char *)pGSAInfo->constellation, (char *)app[0], MAX_STR_LEN);
         pGSAInfo->operating_mode = *((uint8_t*)app[1]);
         pGSAInfo->current_mode = strtol((char *)app[2], NULL, BASE);

         int32_t *sat_prn = pGSAInfo->sat_prn;
         for (int8_t i = 0; i < MAX_SAT_NUM; i++)
         {
            *(&sat_prn[i]) = strtol((char *)app[3+i], NULL, BASE);
         }

         pGSAInfo->pdop = strtof((char *)app[15], NULL);
         pGSAInfo->hdop = strtof((char *)app[16], NULL);
         pGSAInfo->vdop = strtof((char *)app[17], NULL);
         pGSAInfo->checksum = nmea_checksum(app[18]);

         status = PARSE_SUCC;
      }
   }

   return status;
}

int32_t TeseoLIV3F::NMEA_CheckGSAMsg(const char header[])
{
   int32_t is_gsamsg = 1;

   if (strcmp(header, "$GPGSA") == 0)
   {
      is_gsamsg = 0;
   }
   if (strcmp(header, "$GLGSA") == 0)
   {
      is_gsamsg = 0;
   }
   if (strcmp(header, "$GAGSA") == 0)
   {
      is_gsamsg = 0;
   }
   if (strcmp(header, "$BDGSA") == 0)
   {
      is_gsamsg = 0;
   }
   if (strcmp(header, "$GNGSA") == 0)
   {
      is_gsamsg = 0;
   }

   return is_gsamsg;
}


ParseStatus_t TeseoLIV3F::NMEA_ParseGSV(GSV_Info_t *pGSVInfo, uint8_t NMEA[])
{
   int8_t app_idx;
   int32_t gsv_idx = 0;
   int32_t new_field;
   BOOL valid_gsv_msg = FALSE;

   ParseStatus_t status = PARSE_FAIL;

   if(NMEA != NULL)
   {

      /* clear the app[][] buffer */
      for (int8_t i = 0; i < MAX_MSG_LEN; i++)
      {
         (void)memset(app[i], 0, (size_t)MAX_FIELD_LENGTH);
      }

      for (int32_t i = 0, j = 0, k = 0; (NMEA[i] != (uint8_t)'\n'); i++)
      {
         new_field = 0;

         if ((NMEA[i] == (uint8_t)',') || (NMEA[i] == (uint8_t)'*'))
         {
            app[j][k] = (uint8_t)'\0';
            new_field = 1;

            if (NMEA_CheckGSVMsg((char *)app[0]) == 0)
            {
               j++;
               k = 0;
               valid_gsv_msg = TRUE;
            }
            else
            {
               break;
            }
         }
         if(new_field == 0)
         {
            app[j][k] = NMEA[i];
            k++;
         }
      }

      if (valid_gsv_msg == TRUE)
      {
         NMEA_ResetGSVMsg(pGSVInfo);

         (void)strncpy((char *)pGSVInfo->constellation, (char *)app[0], MAX_STR_LEN);
         pGSVInfo->amount = strtol((char *)app[1], NULL, BASE);
         pGSVInfo->number = strtol((char *)app[2], NULL, BASE);
         pGSVInfo->tot_sats = strtol((char *)app[3], NULL, BASE);
         app_idx = 4;
         for (int8_t i = 1; i <= GSV_MSG_SATS; i++)
         {
            pGSVInfo->gsv_sat_i[gsv_idx].prn = strtol((char *)app[app_idx*i], NULL, BASE);
            pGSVInfo->gsv_sat_i[gsv_idx].elev = strtol((char *)app[(app_idx*i)+1], NULL, BASE);
            pGSVInfo->gsv_sat_i[gsv_idx].azim = strtol((char *)app[(app_idx*i)+2], NULL, BASE);
            pGSVInfo->gsv_sat_i[gsv_idx].cn0 = strtol((char *)app[(app_idx*i)+3], NULL, BASE);

            if(pGSVInfo->gsv_sat_i[gsv_idx].prn != 0)
            {
               pGSVInfo->current_sats++;
            }
            gsv_idx++;
         }

         status = PARSE_SUCC;
      }

   }

   return status;
}


int32_t TeseoLIV3F::NMEA_CheckGSVMsg(const char header[])
{
   int32_t is_gsvmsg = 1;

   if (strcmp(header, "$GPGSV") == 0)
   {
      is_gsvmsg = 0;
   }
   if (strcmp(header, "$GLGSV") == 0)
   {
      is_gsvmsg = 0;
   }
   if (strcmp(header, "$GAGSV") == 0)
   {
      is_gsvmsg = 0;
   }
   if (strcmp(header, "$BDGSV") == 0)
   {
      is_gsvmsg = 0;
   }
   if (strcmp(header, "$QZGSV") == 0)
   {
      is_gsvmsg = 0;
   }
   if (strcmp(header, "$GNGSV") == 0)
   {
      is_gsvmsg = 0;
   }

   return is_gsvmsg;
}

void TeseoLIV3F::NMEA_ResetGSVMsg(GSV_Info_t *pGSVInfo)
{
   (void)memset(pGSVInfo->constellation, 0, (size_t)MAX_STR_LEN);
   pGSVInfo->amount = 0;
   pGSVInfo->number = 0;
   pGSVInfo->current_sats = 0;
   pGSVInfo->tot_sats = 0;
   for (int8_t i = 0; i < MAX_SAT_NUM; i++)
   {
      (void)memset(&pGSVInfo->gsv_sat_i[i], 0, sizeof(GSV_SAT_Info_t));
   }
}


ParseStatus_t TeseoLIV3F::NMEA_ParsePSTMVER(PSTMVER_Info_t *pPSTMVER, uint8_t NMEA[])
{
   int8_t new_field;
   BOOL valid_msg = FALSE;

   ParseStatus_t status = PARSE_FAIL;

   if(NMEA != NULL)
   {

      /* clear the app[][] buffer */
      for (int8_t i = 0; i < MAX_MSG_LEN; i++)
      {
         (void)memset(app[i], 0, (size_t)MAX_FIELD_LENGTH);
      }

      for(int32_t i = 0, j = 0, k = 0; (NMEA[i] != (uint8_t)'\n'); i++)
      {
         new_field = 0;

         if ((NMEA[i] == (uint8_t)',') || (NMEA[i] == (uint8_t)'*'))
         {
            app[j][k] = (uint8_t)'\0';
            new_field = 1;

            if (strcmp((char *)app[0], "$PSTMVER") == 0)
            {
               j++;
               k = 0;
               valid_msg = TRUE;
            }
            else
            {
               break;
            }
         }
         if(new_field == 0)
         {
            app[j][k] = NMEA[i];
            k++;
         }
      }

      if (valid_msg == TRUE)
      {
         (void)strncpy((char *)pPSTMVER->pstmver_string, (char *)app[1], MAX_STR_LEN);

         status = PARSE_SUCC;
      }
   }
   return status;
}


ParseStatus_t TeseoLIV3F::NMEA_ParsePSTMGeofence(Geofence_Info_t *pGeofence, uint8_t NMEA[])
{
   int32_t new_field;
   BOOL valid_msg = FALSE;

   ParseStatus_t status = PARSE_FAIL;

   if(NMEA != NULL)
   {

      /* clear the app[][] buffer */
      for (int8_t i = 0; i < MAX_MSG_LEN; i++)
      {
         (void)memset(app[i], 0, (size_t)MAX_FIELD_LENGTH);
      }

      for(int32_t i = 0, j = 0, k = 0; (NMEA[i] != (uint8_t)'\n'); i++)
      {
         new_field = 0;

         if ((NMEA[i] == (uint8_t)',') || (NMEA[i] == (uint8_t)'*'))
         {
            app[j][k] = (uint8_t)'\0';
            new_field = 1;

            if (NMEA_CheckGeofenceMsg((char *)app[0]) == 0)
            {
               j++;
               k = 0;
               valid_msg = TRUE;
            }
            else
            {
               break;
            }
         }
         if(new_field == 0)
         {
            app[j][k] = NMEA[i];
            k++;
         }
      }

      if (valid_msg == TRUE)
      {
         /* Enabling */
         if (strcmp((char *)app[0], "$PSTMCFGGEOFENCEOK") == 0)
         {
            pGeofence->op = GNSS_FEATURE_EN_MSG;
            pGeofence->result = GNSS_OP_OK;
         }
         else if (strcmp((char *)app[0], "$PSTMCFGGEOFENCEERROR") == 0)
         {
            pGeofence->op = GNSS_FEATURE_EN_MSG;
            pGeofence->result = GNSS_OP_ERROR;
         }
         /* Configuring */
         else if (strcmp((char *)app[0], "$PSTMGEOFENCECFGOK") == 0)
         {
            pGeofence->op = GNSS_GEOFENCE_CFG_MSG;
            pGeofence->result = GNSS_OP_OK;
         }
         else if (strcmp((char *)app[0], "$PSTMGEOFENCECFGERROR") == 0)
         {
            pGeofence->op = GNSS_GEOFENCE_STATUS_MSG;
            pGeofence->result = GNSS_OP_ERROR;
         }
         /* Querying Status */
         else if (strcmp((char *)app[0], "$PSTMGEOFENCESTATUS") == 0)
         {
            pGeofence->op = GNSS_GEOFENCE_STATUS_MSG;
            scan_timestamp_time(app[1], &pGeofence->timestamp);
            scan_timestamp_date(app[2], &pGeofence->timestamp);

            int32_t *geofence_status = pGeofence->status;
            for(uint8_t i = 0; i < MAX_GEOFENCES_NUM; i++)
            {
               *(&geofence_status[i]) = strtol((char *)app[3+i], NULL, BASE);
            }
         }
         /* Alarm Msg */
         else if (strcmp((char *)app[0], "$PSTMGEOFENCE") == 0)
         {
            pGeofence->op = GNSS_GEOFENCE_ALARM_MSG;
            scan_timestamp_time(app[1], &pGeofence->timestamp);
            scan_timestamp_date(app[2], &pGeofence->timestamp);
            pGeofence->idAlarm = strtol((char *)app[3], NULL, BASE);
            pGeofence->coords.lat = strtod((char *)app[4], NULL);
            pGeofence->coords.lon = strtod((char *)app[5], NULL);
            pGeofence->coords.radius = strtod((char *)app[6], NULL);
            pGeofence->coords.distance = strtod((char *)app[7], NULL);
            pGeofence->coords.tolerance = strtod((char *)app[8], NULL);
            pGeofence->status[pGeofence->idAlarm] = strtol((char *)app[9], NULL, BASE);
         }
         else
         {
            /* do nothing */
         }

         status = PARSE_SUCC;
      }
   }

   return status;
}

int32_t TeseoLIV3F::NMEA_CheckGeofenceMsg(const char header[])
{
   int32_t is_geofencemsg = 1;

   if (strcmp(header, "$PSTMCFGGEOFENCEOK") == 0)
   {
      is_geofencemsg = 0;
   }
   if (strcmp(header, "$PSTMCFGGEOFENCEERROR") == 0)
   {
      is_geofencemsg = 0;
   }
   if (strcmp(header, "$PSTMGEOFENCECFGOK") == 0)
   {
      is_geofencemsg = 0;
   }
   if (strcmp(header, "$PSTMGEOFENCECFGERROR") == 0)
   {
      is_geofencemsg = 0;
   }
   if (strcmp(header, "$PSTMGEOFENCESTATUS") == 0)
   {
      is_geofencemsg = 0;
   }
   if (strcmp(header, "$PSTMGEOFENCE") == 0)
   {
      is_geofencemsg = 0;
   }
   if (strcmp(header, "$PSTMGEOFENCEREQERROR") == 0)
   {
      is_geofencemsg = 0;
   }

   return is_geofencemsg;
}


void TeseoLIV3F::scan_timestamp_time(uint8_t buf[], Timestamp_Info_t *pTimestamp)
{
   /* FORMAT: HHMMSS */
   pTimestamp->hh = digit2int(buf, 0, TENS);
   pTimestamp->mm = digit2int(buf, 2, TENS);
   pTimestamp->ss = digit2int(buf, 4, TENS);
}

void TeseoLIV3F::scan_timestamp_date(uint8_t buf[], Timestamp_Info_t *pTimestamp)
{
   /* FORMAT: YYYYMMDD */
   pTimestamp->year = digit2int(buf, 0, THOUSANDS);
   pTimestamp->month = digit2int(buf, 4, TENS);
   pTimestamp->day = digit2int(buf, 6, TENS);
}

uint32_t TeseoLIV3F::digit2int(uint8_t buf[], int32_t offset, Decimal_t d)
{
   uint32_t ret = (unsigned char)0;
   uint32_t hu, hd, hc, hm;

   switch (d)
   {
   case TENS:
      hd = char2int(buf[offset]);
      hu = char2int(buf[offset+1]);

      ret = (hd * (unsigned)10) + hu;
      break;

   case HUNDREDS:
      hc = char2int(buf[offset]);
      hd = char2int(buf[offset+1]);
      hu = char2int(buf[offset+2]);

      ret = (hc * (unsigned)100) + (hd * (unsigned)10) + hu;
      break;

   case THOUSANDS:
      hm = char2int(buf[offset]);
      hc = char2int(buf[offset+1]);
      hd = char2int(buf[offset+2]);
      hu = char2int(buf[offset+3]);

      ret = (hm * (unsigned)1000) + (hc * (unsigned)100) + (hd * (unsigned)10) + hu;
      break;

   default:
      break;
   }

   return ret;
}

ParseStatus_t TeseoLIV3F::NMEA_ParsePSTMOdo(Odometer_Info_t *pOdo, uint8_t NMEA[])
{
   int32_t new_field;
   BOOL valid_msg = FALSE;

   ParseStatus_t status = PARSE_FAIL;

   if(NMEA != NULL)
   {

      /* clear the app[][] buffer */
      for (int8_t i = 0; i < MAX_MSG_LEN; i++)
      {
         (void)memset(app[i], 0, (size_t)MAX_FIELD_LENGTH);
      }

      for(int32_t i = 0, j = 0, k = 0; (NMEA[i] != (uint8_t)'\n'); i++)
      {
         new_field = 0;

         if ((NMEA[i] == (uint8_t)',') || (NMEA[i] == (uint8_t)'*'))
         {
            app[j][k] = (uint8_t)'\0';
            new_field = 1;

            if (NMEA_CheckOdoMsg((char *)app[0]) == 0)
            {
               j++;
               k = 0;
               valid_msg = TRUE;
            }
            else
            {
               break;
            }
         }
         if(new_field == 0)
         {
            app[j][k] = NMEA[i];
            k++;
         }
      }

      if (valid_msg == TRUE)
      {
         /* Enabling */
         if (strcmp((char *)app[0], "$PSTMCFGODOOK") == 0)
         {
            pOdo->op = GNSS_FEATURE_EN_MSG;
            pOdo->result = GNSS_OP_OK;
         }
         else if (strcmp((char *)app[0], "$PSTMCFGODOERROR") == 0)
         {
            pOdo->op = GNSS_FEATURE_EN_MSG;
            pOdo->result = GNSS_OP_ERROR;
         }
         /* Start */
         else if (strcmp((char *)app[0], "$PSTMODOSTARTOK") == 0)
         {
            pOdo->op = GNSS_ODO_START_MSG;
            pOdo->result = GNSS_OP_OK;
         }
         else if (strcmp((char *)app[0], "$PSTMODOSTARTERROR") == 0)
         {
            pOdo->op = GNSS_ODO_START_MSG;
            pOdo->result = GNSS_OP_ERROR;
         }
         /* Stop */
         else if (strcmp((char *)app[0], "$PSTMODOSTOPOK") == 0)
         {
            pOdo->op = GNSS_ODO_STOP_MSG;
            pOdo->result = GNSS_OP_OK;
         }
         else if (strcmp((char *)app[0], "$PSTMODOSTOPERROR") == 0)
         {
            pOdo->op = GNSS_ODO_STOP_MSG;
            pOdo->result = GNSS_OP_ERROR;
         }
         else
         {
            /* do nothing */
         }

         status = PARSE_SUCC;
      }
   }
   return status;
}

int32_t TeseoLIV3F::NMEA_CheckOdoMsg(const char header[])
{
   int32_t is_odomsg = 1;

   if (strcmp(header, "$PSTMCFGODOOK") == 0)
   {
      is_odomsg = 0;
   }
   if (strcmp(header, "$PSTMCFGODOERROR") == 0)
   {
      is_odomsg = 0;
   }
   if (strcmp(header, "$PSTMODOSTARTOK") == 0)
   {
      is_odomsg = 0;
   }
   if (strcmp(header, "$PSTMODOSTARTERROR") == 0)
   {
      is_odomsg = 0;
   }
   if (strcmp(header, "$PSTMODOSTOPOK") == 0)
   {
      is_odomsg = 0;
   }
   if (strcmp(header, "$PSTMODOSTOPERROR") == 0)
   {
      is_odomsg = 0;
   }

   return is_odomsg;
}

ParseStatus_t TeseoLIV3F::NMEA_ParsePSTMDatalog(Datalog_Info_t *pDatalog, uint8_t NMEA[])
{
   int32_t new_field;
   BOOL valid_msg = FALSE;

   ParseStatus_t status = PARSE_FAIL;

   if(NMEA != NULL)
   {

      /* clear the app[][] buffer */
      for (int8_t i = 0; i < MAX_MSG_LEN; i++)
      {
         (void)memset(app[i], 0, (size_t)MAX_FIELD_LENGTH);
      }

      for(int32_t i = 0, j = 0, k = 0; (NMEA[i] != (uint8_t)'\n'); i++)
      {
         new_field = 0;

         if ((NMEA[i] == (uint8_t)',') || (NMEA[i] == (uint8_t)'*'))
         {
            app[j][k] = (uint8_t)'\0';
            new_field = 1;

            if (NMEA_CheckDatalogMsg((char *)app[0]) == 0)
            {
               j++;
               k = 0;
               valid_msg = TRUE;
            }
            else
            {
               break;
            }
         }
         if(new_field == 0)
         {
            app[j][k] = NMEA[i];
            k++;
         }
      }

      if (valid_msg == TRUE)
      {
         /* Enabling */
         if (strcmp((char *)app[0], "$PSTMCFGLOGOK") == 0)
         {
            pDatalog->op = GNSS_FEATURE_EN_MSG;
            pDatalog->result = GNSS_OP_OK;
         }
         else if (strcmp((char *)app[0], "$PSTMCFGLOGERROR") == 0)
         {
            pDatalog->op = GNSS_FEATURE_EN_MSG;
            pDatalog->result = GNSS_OP_ERROR;
         }
         /* Configuring */
         else if (strcmp((char *)app[0], "$PSTMLOGCREATEOK") == 0)
         {
            pDatalog->op = GNSS_DATALOG_CFG_MSG;
            pDatalog->result = GNSS_OP_OK;
         }
         else if (strcmp((char *)app[0], "$PSTMLOGCREATEERROR") == 0)
         {
            pDatalog->op = GNSS_DATALOG_CFG_MSG;
            pDatalog->result = GNSS_OP_ERROR;
         }
         /* Start */
         else if (strcmp((char *)app[0], "$PSTMLOGSTARTOK") == 0)
         {
            pDatalog->op = GNSS_DATALOG_START_MSG;
            pDatalog->result = GNSS_OP_OK;
         }
         else if (strcmp((char *)app[0], "$PSTMLOGSTARTERROR") == 0)
         {
            pDatalog->op = GNSS_DATALOG_START_MSG;
            pDatalog->result = GNSS_OP_ERROR;
         }
         /* Stop */
         else if (strcmp((char *)app[0], "$PSTMLOGSTOPOK") == 0)
         {
            pDatalog->op = GNSS_DATALOG_STOP_MSG;
            pDatalog->result = GNSS_OP_OK;
         }
         else if (strcmp((char *)app[0], "$PSTMLOGSTOPERROR") == 0)
         {
            pDatalog->op = GNSS_DATALOG_STOP_MSG;
            pDatalog->result = GNSS_OP_ERROR;
         }
         /* Erase */
         else if (strcmp((char *)app[0], "$PSTMLOGERASEOK") == 0)
         {
            pDatalog->op = GNSS_DATALOG_ERASE_MSG;
            pDatalog->result = GNSS_OP_OK;
         }
         else if (strcmp((char *)app[0], "$PSTMLOGERASEERROR") == 0)
         {
            pDatalog->op = GNSS_DATALOG_ERASE_MSG;
            pDatalog->result = GNSS_OP_ERROR;
         }
         else
         {
            /* do nothing */
         }

         status = PARSE_SUCC;
      }
   }
   return status;
}

int32_t TeseoLIV3F::NMEA_CheckDatalogMsg(const char header[])
{
   int32_t is_datalogmsg = 1;

   if (strcmp(header, "$PSTMCFGLOGOK") == 0)
   {
      is_datalogmsg = 0;
   }
   if (strcmp(header, "$PSTMCFGLOGERROR") == 0)
   {
      is_datalogmsg = 0;
   }
   if (strcmp(header, "$PSTMLOGCREATEOK") == 0)
   {
      is_datalogmsg = 0;
   }
   if (strcmp(header, "$PSTMLOGCREATEERROR") == 0)
   {
      is_datalogmsg = 0;
   }
   if (strcmp(header, "$PSTMLOGSTARTOK") == 0)
   {
      is_datalogmsg = 0;
   }
   if (strcmp(header, "$PSTMLOGSTARTERROR") == 0)
   {
      is_datalogmsg = 0;
   }
   if (strcmp(header, "$PSTMLOGSTOPOK") == 0)
   {
      is_datalogmsg = 0;
   }
   if (strcmp(header, "$PSTMLOGSTOPERROR") == 0)
   {
      is_datalogmsg = 0;
   }
   if (strcmp(header, "$PSTMLOGERASEOK") == 0)
   {
      is_datalogmsg = 0;
   }
   if (strcmp(header, "$PSTMLOGERASEERROR") == 0)
   {
      is_datalogmsg = 0;
   }

   return is_datalogmsg;
}

ParseStatus_t TeseoLIV3F::NMEA_ParsePSTMsgl(OpResult_t *pResult, uint8_t NMEA[])
{
   int32_t new_field;
   BOOL valid_msg = FALSE;

   ParseStatus_t status = PARSE_FAIL;

   if(NMEA != NULL)
   {

      /* clear the app[][] buffer */
      for (int8_t i = 0; i < MAX_MSG_LEN; i++)
      {
         (void)memset(app[i], 0, (size_t)MAX_FIELD_LENGTH);
      }

      for(int32_t i = 0, j = 0, k = 0; (NMEA[i] != (uint8_t)'\n'); i++)
      {
         new_field = 0;

         if ((NMEA[i] == (uint8_t)',') || (NMEA[i] == (uint8_t)'*'))
         {
            app[j][k] = (uint8_t)'\0';
            new_field = 1;

            if (NMEA_CheckListMsg((char *)app[0]) == 0)
            {
               j++;
               k = 0;
               valid_msg = TRUE;
            }
            else
            {
               break;
            }
         }
         if(new_field == 0)
         {
            app[j][k] = NMEA[i];
            k++;
         }
      }

      if (valid_msg == TRUE)
      {
         /* Enabling */
         if (strcmp((char *)app[0], "$PSTMCFGMSGLOK") == 0)
         {
            *pResult = GNSS_OP_OK;
         }
         else if (strcmp((char *)app[0], "$PSTMCFGMSGLERROR") == 0)
         {
            *pResult = GNSS_OP_ERROR;
         }
         else
         {
            /* do nothing */
         }

         status = PARSE_SUCC;
      }

   }
   return status;
}

int32_t TeseoLIV3F::NMEA_CheckListMsg(const char header[])
{
   int32_t is_listmsg = 1;

   if (strcmp(header, "$PSTMCFGMSGLOK") == 0)
   {
      is_listmsg = 0;
   }
   if (strcmp(header, "$PSTMCFGMSGLERROR") == 0)
   {
      is_listmsg = 0;
   }

   return is_listmsg;
}

ParseStatus_t TeseoLIV3F::NMEA_ParsePSTMSavePar(OpResult_t *pResult, uint8_t NMEA[])
{
   int32_t new_field;
   BOOL valid_msg = FALSE;

   ParseStatus_t status = PARSE_FAIL;

   if(NMEA != NULL)
   {

      /* clear the app[][] buffer */
      for (int8_t i = 0; i < MAX_MSG_LEN; i++)
      {
         (void)memset(app[i], 0, (size_t)MAX_FIELD_LENGTH);
      }

      for(int32_t i = 0, j = 0, k = 0; (NMEA[i] != (uint8_t)'\n'); i++)
      {
         new_field = 0;

         if ((NMEA[i] == (uint8_t)',') || (NMEA[i] == (uint8_t)'*'))
         {
            app[j][k] = (uint8_t)'\0';
            new_field = 1;

            if (NMEA_CheckSaveparMsg((char *)app[0]) == 0)
            {
               j++;
               k = 0;
               valid_msg = TRUE;
            }
            else
            {
               break;
            }
         }
         if(new_field == 0)
         {
            app[j][k] = NMEA[i];
            k++;
         }
      }

      if (valid_msg == TRUE)
      {
         if (strcmp((char *)app[0], "$PSTMSAVEPAROK") == 0)
         {
            *pResult = GNSS_OP_OK;
         }
         else if (strcmp((char *)app[0], "$PSTMSAVEPARERROR") == 0)
         {
            *pResult = GNSS_OP_ERROR;
         }
         else
         {
            /* do nothing */
         }

         status = PARSE_SUCC;
      }
   }
   return status;
}

int32_t TeseoLIV3F::NMEA_CheckSaveparMsg(const char header[])
{
   int32_t is_savevarmsg = 1;

   if (strcmp(header, "$PSTMSAVEPAROK") == 0)
   {
      is_savevarmsg = 0;
   }
   if (strcmp(header, "$PSTMSAVEPARERROR") == 0)
   {
      is_savevarmsg = 0;
   }

   return is_savevarmsg;
}

ParseStatus_t TeseoLIV3F::NMEA_ParsePSTMPASSRTN(PSTMPASSRTN_Info_t *pPSTMPASSRTN, uint8_t NMEA[])
{
   int8_t new_field;
   BOOL valid_msg = FALSE;

   ParseStatus_t status = PARSE_FAIL;

   if(NMEA != NULL)
   {

      /* clear the app[][] buffer */
      for (int8_t i = 0; i < MAX_MSG_LEN; i++)
      {
         (void)memset(app[i], 0, (size_t)MAX_FIELD_LENGTH);
      }

      for(int32_t i = 0, j = 0, k = 0; (NMEA[i] != (uint8_t)'\n'); i++)
      {
         new_field = 0;

         if ((NMEA[i] == (uint8_t)',') || (NMEA[i] == (uint8_t)'*'))
         {
            app[j][k] = (uint8_t)'\0';
            new_field = 1;

            if (NMEA_CheckPassMsg((char *)app[0]) == 0)
            {
               j++;
               k = 0;
               valid_msg = TRUE;
            }
            else
            {
               break;
            }
         }
         if(new_field == 0)
         {
            app[j][k] = NMEA[i];
            k++;
         }
      }

      if (valid_msg == TRUE)
      {
         if (strcmp((char *)app[0], "$PSTMSTAGPS8PASSRTN") == 0)
         {
            (void)strncpy((char *)pPSTMPASSRTN->deviceId, (char *)app[1], MAX_STR_LEN);
            (void)strncpy((char *)pPSTMPASSRTN->pwd, (char *)app[2], 64);
            pPSTMPASSRTN->result = GNSS_OP_OK;
         }
         else if (strcmp((char *)app[0], "$PSTMSTAGPS8PASSGENERROR") == 0)
         {
            pPSTMPASSRTN->result = GNSS_OP_ERROR;
         }
         else
         {
            /* do nothing */
         }

         status = PARSE_SUCC;
      }
   }
   return status;
}

int32_t TeseoLIV3F::NMEA_CheckPassMsg(const char header[])
{
   int32_t is_passmsg = 1;

   if (strcmp(header, "$PSTMSTAGPS8PASSRTN") == 0)
   {
      is_passmsg = 0;
   }
   if (strcmp(header, "$PSTMSTAGPS8PASSGENERROR") == 0)
   {
      is_passmsg = 0;
   }

   return is_passmsg;
}

ParseStatus_t TeseoLIV3F::NMEA_ParsePSTMAGPS(PSTMAGPS_Info_t *pPSTMAGPS, uint8_t NMEA[])
{
   int8_t new_field;
   BOOL valid_msg = FALSE;

   ParseStatus_t status = PARSE_FAIL;

   if(NMEA != NULL)
   {
      /* clear the app[][] buffer */
      for (int8_t i = 0; i < MAX_MSG_LEN; i++)
      {
         (void)memset(app[i], 0, (size_t)MAX_FIELD_LENGTH);
      }

      for(int32_t i = 0, j = 0, k = 0; (NMEA[i] != (uint8_t)'\n'); i++)
      {
         new_field = 0;

         if ((NMEA[i] == (uint8_t)',') || (NMEA[i] == (uint8_t)'*'))
         {
            app[j][k] = (uint8_t)'\0';
            new_field = 1;

            if (NMEA_CheckAGPSMsg((char *)app[0]) == 0)
            {
               j++;
               k = 0;
               valid_msg = TRUE;
            }
            else
            {
               break;
            }
         }
         if(new_field == 0)
         {
            app[j][k] = NMEA[i];
            k++;
         }
      }

      if (valid_msg == TRUE)
      {
         /* Status */
         if (strcmp((char *)app[0], "$PSTMAGPSSTATUS") == 0)
         {
            pPSTMAGPS->op = GNSS_AGPS_STATUS_MSG;
            pPSTMAGPS->status = strtol((char *)app[1], NULL, BASE);
         }
         /* Begin */
         else if (strcmp((char *)app[0], "$PSTMSTAGPSSEEDBEGINOK") == 0)
         {
            pPSTMAGPS->op = GNSS_AGPS_BEGIN_MSG;
            pPSTMAGPS->result = GNSS_OP_OK;
         }
         else if (strcmp((char *)app[0], "$PSTMSTAGPSSEEDBEGINERROR") == 0)
         {
            pPSTMAGPS->op = GNSS_AGPS_BEGIN_MSG;
            pPSTMAGPS->result = GNSS_OP_ERROR;
         }
         /* Block type */
         else if (strcmp((char *)app[0], "$PSTMSTAGPSBLKTYPEOK") == 0)
         {
            pPSTMAGPS->op = GNSS_AGPS_BLKTYPE_MSG;
            pPSTMAGPS->result = GNSS_OP_OK;
         }
         else if (strcmp((char *)app[0], "$PSTMSTAGPSBLKTYPEERROR") == 0)
         {
            pPSTMAGPS->op = GNSS_AGPS_BLKTYPE_MSG;
            pPSTMAGPS->result = GNSS_OP_ERROR;
         }
         /* Slot freq */
         else if (strcmp((char *)app[0], "$PSTMSTAGPSSLOTFRQOK") == 0)
         {
            pPSTMAGPS->op = GNSS_AGPS_SLOTFRQ_MSG;
            pPSTMAGPS->result = GNSS_OP_OK;
         }
         else if (strcmp((char *)app[0], "$PSTMSTAGPSSLOTFRQERROR") == 0)
         {
            pPSTMAGPS->op = GNSS_AGPS_SLOTFRQ_MSG;
            pPSTMAGPS->result = GNSS_OP_ERROR;
         }
         /* Seed pkt */
         else if (strcmp((char *)app[0], "$PSTMSTAGPSSEEDPKTOK") == 0)
         {
            pPSTMAGPS->op = GNSS_AGPS_SEEDPKT_MSG;
            pPSTMAGPS->result = GNSS_OP_OK;
         }
         else if (strcmp((char *)app[0], "$PSTMSTAGPSSEEDPKTERROR") == 0)
         {
            pPSTMAGPS->op = GNSS_AGPS_SEEDPKT_MSG;
            pPSTMAGPS->result = GNSS_OP_ERROR;
         }
         /* Propagate */
         else if (strcmp((char *)app[0], "$PSTMSTAGPSSEEDPROPOK") == 0)
         {
            pPSTMAGPS->op = GNSS_AGPS_PROP_MSG;
            pPSTMAGPS->result = GNSS_OP_OK;
         }
         else if (strcmp((char *)app[0], "$PSTMSTAGPSSEEDPROPERROR") == 0)
         {
            pPSTMAGPS->op = GNSS_AGPS_PROP_MSG;
            pPSTMAGPS->result = GNSS_OP_ERROR;
         }
         /* Init time */
         else if (strcmp((char *)app[0], "$PSTMINITTIMEOK") == 0)
         {
            pPSTMAGPS->op = GNSS_AGPS_INITTIME_MSG;
            pPSTMAGPS->result = GNSS_OP_OK;
         }
         else if (strcmp((char *)app[0], "$PSTMINITTIMEERROR") == 0)
         {
            pPSTMAGPS->op = GNSS_AGPS_INITTIME_MSG;
            pPSTMAGPS->result = GNSS_OP_ERROR;
         }
         else
         {
            /* do nothing */
         }

         status = PARSE_SUCC;
      }
   }
   return status;
}

int32_t TeseoLIV3F::NMEA_CheckAGPSMsg(const char header[])
{
   int32_t is_passmsg = 1;

   /* Status */
   if (strcmp(header, "$PSTMAGPSSTATUS") == 0)
   {
      is_passmsg = 0;
   }
   /* Begin */
   if (strcmp(header, "$PSTMSTAGPSSEEDBEGINOK") == 0)
   {
      is_passmsg = 0;
   }
   if (strcmp(header, "$PSTMSTAGPSSEEDBEGINERROR") == 0)
   {
      is_passmsg = 0;
   }
   /* Block type */
   if (strcmp(header, "$PSTMSTAGPSBLKTYPEOK") == 0)
   {
      is_passmsg = 0;
   }
   if (strcmp(header, "$PSTMSTAGPSBLKTYPEERROR") == 0)
   {
      is_passmsg = 0;
   }
   /* Slot freq */
   if (strcmp(header, "$PSTMSTAGPSSLOTFRQOK") == 0)
   {
      is_passmsg = 0;
   }
   if (strcmp(header, "$PSTMSTAGPSSLOTFRQERROR") == 0)
   {
      is_passmsg = 0;
   }
   /* Seed pkt */
   if (strcmp(header, "$PSTMSTAGPSSEEDPKTOK") == 0)
   {
      is_passmsg = 0;
   }
   if (strcmp(header, "$PSTMSTAGPSSEEDPKTERROR") == 0)
   {
      is_passmsg = 0;
   }
   /* Propagate */
   if (strcmp(header, "$PSTMSTAGPSSEEDPROPOK") == 0)
   {
      is_passmsg = 0;
   }
   if (strcmp(header, "$PSTMSTAGPSSEEDPROPERROR") == 0)
   {
      is_passmsg = 0;
   }
   /* Init time */
   if (strcmp(header, "$PSTMINITTIMEOK") == 0)
   {
      is_passmsg = 0;
   }
   if (strcmp(header, "$PSTMINITTIMEERROR") == 0)
   {
      is_passmsg = 0;
   }
   return is_passmsg;
}

void TeseoLIV3F::NMEA_Copy_Data(GPGGA_Info_t *pInfo, GPGGA_Info_t GPGGAInfo)
{
   pInfo->acc          = GPGGAInfo.acc;
   pInfo->geoid.height = GPGGAInfo.geoid.height;
   pInfo->geoid.mis    = GPGGAInfo.geoid.mis;
   pInfo->sats         = GPGGAInfo.sats;
   pInfo->update       = GPGGAInfo.update;
   pInfo->utc.hh       = GPGGAInfo.utc.hh;
   pInfo->utc.mm       = GPGGAInfo.utc.mm;
   pInfo->utc.ss       = GPGGAInfo.utc.ss;
   pInfo->utc.utc      = GPGGAInfo.utc.utc;
   pInfo->valid        = GPGGAInfo.valid;
   pInfo->xyz.alt      = GPGGAInfo.xyz.alt;
   pInfo->xyz.lat      = GPGGAInfo.xyz.lat;
   pInfo->xyz.lon      = GPGGAInfo.xyz.lon;
   pInfo->xyz.ew       = GPGGAInfo.xyz.ew;
   pInfo->xyz.ns       = GPGGAInfo.xyz.ns;
   pInfo->xyz.mis      = GPGGAInfo.xyz.mis;
   pInfo->checksum     = GPGGAInfo.checksum;
}


uint32_t TeseoLIV3F::char2int(uint8_t c)
{
   uint32_t ret = (unsigned char)0;

   if((c >= (uint8_t)'0') && (c <= (uint8_t)'9'))
   {
      ret = (unsigned char)(c - (uint8_t)'0');
   }

   if((c >= (uint8_t)'A') && (c <= (uint8_t)'F'))
   {
      ret = (unsigned char)(c - (uint8_t)'A') + (unsigned)10;
   }

   if((c >= (uint8_t)'a') && (c <= (uint8_t)'f'))
   {
      ret = (unsigned char)(c - (uint8_t)'a') + (unsigned)10;
   }

   return ret;
}


