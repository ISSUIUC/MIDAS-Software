/**
 * @file telemetry.cpp
 *
 * @brief This file defines the telemetry class used to facilitate
 * telemetry commands and data transfer between the on-board flight
 * computer and the ground station.
 *
 * Spaceshot Avionics 2023-24
 * Illinois Space Society - Software Team
 * Gautam Dayal
 * Nicholas Phillips
 * Patrick Marschoun
 * Peter Giannetos
 * Rishi Gokkumutkkala
 * Aaditya Voruganti
 * Magilan Sendhil
 */

#include "hardware/telemetry_backend.h"
#include "hardware/pins.h"

// Change to 434.0 or other frequency, must match RX's freq!
#ifdef IS_BOOSTER
#define TX_FREQ 425.15
#else
#define TX_FREQ 426.15
#endif

static RadioEvents_t RadioEvents;

#define RF_FREQUENCY 430000000  // Hz (430 MHz)
#define TX_OUTPUT_POWER 22		// dBm
#define LORA_BANDWIDTH 0		// [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 8// [SF7..SF12]
#define LORA_CODINGRATE 4		// [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
#define LORA_PREAMBLE_LENGTH 10  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0   // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 1000
#define TX_TIMEOUT_VALUE 1000
#define LORA_BUFFER_SIZE 64 // Define the payload size here

/**
 * @brief Default constructor for the telemetry system
*/
TelemetryBackend::TelemetryBackend() {
    led_state = false;
}

uint8_t lora_payload[255] = {0};
bool tx_done = false;
bool rx_done = false;

namespace {
// Lora callbacks
void OnTxDone(void)
{
    Serial.println("Tx done!");
	tx_done = true;
    Radio.Rx(RX_TIMEOUT_VALUE);
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{

    memcpy(lora_payload, payload, size);
    rx_done = false;
}

void OnTxTimeout(void)
{
    rx_done = true;
    Radio.Rx(RX_TIMEOUT_VALUE);
}

void OnRxTimeout(void)
{
    Radio.Rx(RX_TIMEOUT_VALUE);
}

void OnRxError(void)
{
	Serial.println("RX ERR!");
}

void OnCadDone(bool cadResult)
{
}
}

/**
 * @brief Initializes the telemetry system
 * 
 * @return Error Code
*/
ErrorCode TelemetryBackend::init() {
    pinMode(E22_RESET, OUTPUT);
    digitalWrite(E22_RESET, HIGH);
    delay(100);
    digitalWrite(E22_RESET, LOW);
    delay(100);
    digitalWrite(E22_RESET, HIGH);
    delay(5);

    hwConfig.CHIP_TYPE = SX1262_CHIP;		  // Example uses an eByte E22 module with an SX1262
	hwConfig.PIN_LORA_RESET = E22_RESET; // LORA RESET
	hwConfig.PIN_LORA_NSS = E22_CS;	  // LORA SPI CS
	hwConfig.PIN_LORA_SCLK = SPI_SCK;	  // LORA SPI CLK
	hwConfig.PIN_LORA_MISO = SPI_MISO;	  // LORA SPI MISO
	hwConfig.PIN_LORA_DIO_1 = E22_DI01; // LORA DIO_1
	hwConfig.PIN_LORA_BUSY = E22_BUSY;	  // LORA SPI BUSY
	hwConfig.PIN_LORA_MOSI = SPI_MOSI;	  // LORA SPI MOSI
	hwConfig.RADIO_RXEN = E22_RXEN;		  // LORA ANTENNA RX ENABLE
	hwConfig.USE_DIO2_ANT_SWITCH = true;	  // Example uses an CircuitRocks Alora RFM1262 which uses DIO2 pins as antenna control
	hwConfig.USE_DIO3_TCXO = false;			  // Example uses an CircuitRocks Alora RFM1262 which uses DIO3 to control oscillator voltage


    if (!rf95.setFrequency(RF95_FREQ)) {
        return ErrorCode::RadioSetFrequencyFailed;
    }
    rf95.setSignalBandwidth(250000);
    rf95.setCodingRate4(8);
    rf95.setSpreadingFactor(8);
    rf95.setPayloadCRC(true);
    /*
     * The default transmitter power is 13dBm, using PA_BOOST.
     * If you are using RFM95/96/97/98 modules which uses the PA_BOOST
     * transmitter pin, then you can set transmitter powers from 5 to 23 dBm:
     */
    rf95.setTxPower(23, false);

	uint32_t err_code = lora_hardware_init(hwConfig);
	if (err_code != 0)
	{
		Serial.printf("lora_hardware_init failed - %d\n", err_code);
		while(1) {};
	}
	Serial.println("Lora hardware init successful");
	
	RadioEvents.TxDone = OnTxDone;
	RadioEvents.RxDone = OnRxDone;
	RadioEvents.TxTimeout = OnTxTimeout;
	RadioEvents.RxTimeout = OnRxTimeout;
	RadioEvents.RxError = OnRxError;
	RadioEvents.CadDone = OnCadDone;
	Serial.println("Lora callbacks set");

	// Initialize the Radio
	Radio.Init(&RadioEvents);

	// Set Radio channel
	Radio.SetChannel((uint32_t) (TX_FREQ * 1e6));
	Serial.println("Lora radio channel init successful");

	// Set Radio TX configuration
	Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
					  LORA_SPREADING_FACTOR, LORA_CODINGRATE,
					  LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
					  true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);

	// Set Radio RX configuration
	Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
					  LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
					  LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
					  0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
    Radio.Rx(RX_TIMEOUT_VALUE);
    return ErrorCode::NoError;
}

/**
 * @brief Gets RSSI of recent packets
 * 
 * @return RSSI of most recent packet
*/
int16_t TelemetryBackend::getRecentRssi() {
    return Radio.Rssi(MODEM_LORA);
}

/**
 * @brief Sets new frequency for the LoRa module
 * 
 * @param freq New frequency to set the LoRa module to
*/
void TelemetryBackend::setFrequency(float freq) {
    Radio.SetChannel((uint32_t) (freq * 1e6));
}
