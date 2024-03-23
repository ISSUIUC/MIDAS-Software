/**
 ******************************************************************************
 * @file    X_NUCLEO_GNSS1A1_VirtualCOMPort.ino
 * @author  AST
 * @version V1.0.0
 * @date    January 2018
 * @brief   Arduino test application for the STMicrolectronics X-NUCLEO-GNSS1A1
 *          GNSS module expansion board based on TeseoLIV3F.
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

//NOTE: this sketch should be uploaded in order to perform a Firmware Upgrade procedure using
//      the Flash Updater Java tool provided at https://github.com/stm32duino/Teseo-LIV3F-Flash-Updater

volatile uint8_t fromPC[16];
volatile uint8_t fromGNSS[16];
volatile unsigned long idxPC = 0;
volatile unsigned long idxGNSS = 0;

#ifdef ARDUINO_ARCH_STM32
HardwareSerial Serial1(PA10, PA9);
#endif

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);
  delay(1000);
  digitalWrite(7,HIGH);
  delay(50);
}


void loop() {
  if(Serial.available()){
    fromPC[idxPC]= Serial.read();
    Serial1.write(fromPC[idxPC]);
    idxPC++;
    idxPC %= 16;
    }
  if(Serial1.available()){
    fromGNSS[idxGNSS]= Serial1.read();
    Serial.write(fromGNSS[idxGNSS]);
    idxGNSS++;
    idxGNSS %= 16;
    }
}
