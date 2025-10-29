#pragma once
#include<cstdint>
#include<stddef.h>
#include<Arduino.h>
#include<SPI.h>


/*!
 * \brief Represents the interruption masks available for the radio
 *
 * \remark Note that not all these interruptions are available for all packet types
 */
typedef enum
{
	IRQ_RADIO_NONE = 0x0000,
	IRQ_TX_DONE = 0x0001,
	IRQ_RX_DONE = 0x0002,
	IRQ_PREAMBLE_DETECTED = 0x0004,
	IRQ_SYNCWORD_VALID = 0x0008,
	IRQ_HEADER_VALID = 0x0010,
	IRQ_HEADER_ERROR = 0x0020,
	IRQ_CRC_ERROR = 0x0040,
	IRQ_CAD_DONE = 0x0080,
	IRQ_CAD_ACTIVITY_DETECTED = 0x0100,
	IRQ_RX_TX_TIMEOUT = 0x0200,
	IRQ_RADIO_ALL = 0xFFFF,
} RadioIrqMasks_t;


typedef enum RadioCommands_e
{
	RADIO_GET_STATUS = 0xC0,
	RADIO_WRITE_REGISTER = 0x0D,
	RADIO_READ_REGISTER = 0x1D,
	RADIO_WRITE_BUFFER = 0x0E,
	RADIO_READ_BUFFER = 0x1E,
	RADIO_SET_SLEEP = 0x84,
	RADIO_SET_STANDBY = 0x80,
	RADIO_SET_FS = 0xC1,
	RADIO_SET_TX = 0x83,
	RADIO_SET_RX = 0x82,
	RADIO_SET_RXDUTYCYCLE = 0x94,
	RADIO_SET_CAD = 0xC5,
	RADIO_SET_TXCONTINUOUSWAVE = 0xD1,
	RADIO_SET_TXCONTINUOUSPREAMBLE = 0xD2,
	RADIO_SET_PACKETTYPE = 0x8A,
	RADIO_GET_PACKETTYPE = 0x11,
	RADIO_SET_RFFREQUENCY = 0x86,
	RADIO_SET_TXPARAMS = 0x8E,
	RADIO_SET_PACONFIG = 0x95,
	RADIO_SET_CADPARAMS = 0x88,
	RADIO_SET_BUFFERBASEADDRESS = 0x8F,
	RADIO_SET_MODULATIONPARAMS = 0x8B,
	RADIO_SET_PACKETPARAMS = 0x8C,
	RADIO_GET_RXBUFFERSTATUS = 0x13,
	RADIO_GET_PACKETSTATUS = 0x14,
	RADIO_GET_RSSIINST = 0x15,
	RADIO_GET_STATS = 0x10,
	RADIO_RESET_STATS = 0x00,
	RADIO_CFG_DIOIRQ = 0x08,
	RADIO_GET_IRQSTATUS = 0x12,
	RADIO_CLR_IRQSTATUS = 0x02,
	RADIO_CALIBRATE = 0x89,
	RADIO_CALIBRATEIMAGE = 0x98,
	RADIO_SET_REGULATORMODE = 0x96,
	RADIO_GET_ERROR = 0x17,
	RADIO_CLR_ERROR = 0x07,
	RADIO_SET_TCXOMODE = 0x97,
	RADIO_SET_TXFALLBACKMODE = 0x93,
	RADIO_SET_RFSWITCHMODE = 0x9D,
	RADIO_SET_STOPRXTIMERONPREAMBLE = 0x9F,
	RADIO_SET_LORASYMBTIMEOUT = 0xA0,
} RadioCommands_t;

/*!
 * \brief Represents the operating mode the radio is actually running
 */
typedef enum
{
	MODE_SLEEP = 0x00, //! The radio is in sleep mode
	MODE_STDBY_RC,	   //! The radio is in standby mode with RC oscillator
	MODE_STDBY_XOSC,   //! The radio is in standby mode with XOSC oscillator
	MODE_FS,		   //! The radio is in frequency synthesis mode
	MODE_TX,		   //! The radio is in transmit mode
	MODE_RX,		   //! The radio is in receive mode
	MODE_RX_DC,		   //! The radio is in receive duty cycle mode
	MODE_CAD		   //! The radio is in channel activity detection mode
} RadioOperatingModes_t;



/*!
 * \brief Represents the bandwidth values for LoRa packet type
 */
typedef enum
{
	LORA_BW_500 = 6,
	LORA_BW_250 = 5,
	LORA_BW_125 = 4,
	LORA_BW_062 = 3,
	LORA_BW_041 = 10,
	LORA_BW_031 = 2,
	LORA_BW_020 = 9,
	LORA_BW_015 = 1,
	LORA_BW_010 = 8,
	LORA_BW_007 = 0,
} RadioLoRaBandwidths_t;

/*!
 * \brief Represents the coding rate values for LoRa packet type
 */
typedef enum
{
	LORA_CR_4_5 = 0x01,
	LORA_CR_4_6 = 0x02,
	LORA_CR_4_7 = 0x03,
	LORA_CR_4_8 = 0x04,
} RadioLoRaCodingRates_t;


/*!
 * \brief Holds the Radio lengths mode for the LoRa packet type
 */
typedef enum
{
	LORA_PACKET_VARIABLE_LENGTH = 0x00, //!< The packet is on variable size, header included
	LORA_PACKET_FIXED_LENGTH = 0x01,	//!< The packet is known on both sides, no header included in the packet
	LORA_PACKET_EXPLICIT = LORA_PACKET_VARIABLE_LENGTH,
	LORA_PACKET_IMPLICIT = LORA_PACKET_FIXED_LENGTH,
} RadioLoRaPacketLengthsMode_t;

/*!
 * \brief Represents the CRC mode for LoRa packet type
 */
typedef enum
{
	LORA_CRC_ON = 0x01,	 //!< CRC activated
	LORA_CRC_OFF = 0x00, //!< CRC not used
} RadioLoRaCrcModes_t;

/*!
 * \brief Represents the IQ mode for LoRa packet type
 */
typedef enum
{
	LORA_IQ_NORMAL = 0x00,
	LORA_IQ_INVERTED = 0x01,
} RadioLoRaIQModes_t;

/*!
 * \brief Represents the voltage used to control the TCXO on/off from DIO3
 */
typedef enum
{
	TCXO_CTRL_1_6V = 0x00,
	TCXO_CTRL_1_7V = 0x01,
	TCXO_CTRL_1_8V = 0x02,
	TCXO_CTRL_2_2V = 0x03,
	TCXO_CTRL_2_4V = 0x04,
	TCXO_CTRL_2_7V = 0x05,
	TCXO_CTRL_3_0V = 0x06,
	TCXO_CTRL_3_3V = 0x07,
} RadioTcxoCtrlVoltage_t;


/*!
 * \brief Represents the ramping time for power amplifier
 */
typedef enum
{
	RADIO_RAMP_10_US = 0x00,
	RADIO_RAMP_20_US = 0x01,
	RADIO_RAMP_40_US = 0x02,
	RADIO_RAMP_80_US = 0x03,
	RADIO_RAMP_200_US = 0x04,
	RADIO_RAMP_800_US = 0x05,
	RADIO_RAMP_1700_US = 0x06,
	RADIO_RAMP_3400_US = 0x07,
} RadioRampTimes_t;

/*!
 * \brief Represents the number of symbols to be used for channel activity detection operation
 */
typedef enum
{
	LORA_CAD_01_SYMBOL = 0x00,
	LORA_CAD_02_SYMBOL = 0x01,
	LORA_CAD_04_SYMBOL = 0x02,
	LORA_CAD_08_SYMBOL = 0x03,
	LORA_CAD_16_SYMBOL = 0x04,
} RadioLoRaCadSymbols_t;

typedef enum
{
	PACKET_TYPE_GFSK = 0x00,
	PACKET_TYPE_LORA = 0x01,
} RadioPacketType_t;

enum class SX1268Error {
	NoError,
	BusyTimeout,
	BadParameter,
	TxTimeout,
	RxTimeout,
	CrcError,
	FailedReadback,
	DeviceError,
	UnknownError,
};

#define REG_OCP 0x08E7
#define XTAL_FREQ (double)32000000
#define FREQ_DIV (double)pow(2.0, 25.0)
#define FREQ_STEP (double)(XTAL_FREQ / FREQ_DIV)
//15.625 us
#define TIME_DIVISION_PER_MS 64

class SX1268 {
public:
	SX1268(SPIClass& spi, uint8_t cs, uint8_t busy, uint8_t dio1, uint8_t rxen, uint8_t reset): 
	spi(spi), pin_cs(cs), pin_busy(busy), pin_dio1(dio1),
	pin_rxen(rxen), pin_reset(reset) {}
	
	[[nodiscard]] SX1268Error setup();
	[[nodiscard]] SX1268Error set_frequency(uint32_t freq);
	[[nodiscard]] SX1268Error set_modulation_params(uint8_t spreading_factor, RadioLoRaBandwidths_t bandwidth, RadioLoRaCodingRates_t cr, bool low_data_rate);
	[[nodiscard]] SX1268Error set_tx_power(int8_t dbm);
	[[nodiscard]] SX1268Error send(uint8_t* data, size_t len);
	[[nodiscard]] SX1268Error recv(uint8_t* data, size_t len, size_t timeout_ms);

private:
	[[nodiscard]] SX1268Error wait_on_busy();
	[[nodiscard]] SX1268Error write_command(RadioCommands_t command, uint8_t* buffer, size_t size);
	[[nodiscard]] SX1268Error read_command(RadioCommands_t command, uint8_t* buffer, size_t size);
	[[nodiscard]] SX1268Error write_buffer(uint8_t offset, const uint8_t* buffer, size_t size);
	[[nodiscard]] SX1268Error read_buffer(uint8_t offset, uint8_t* buffer, size_t size);
	[[nodiscard]] SX1268Error write_registers(uint16_t address, uint8_t* buffer, size_t size);
	[[nodiscard]] SX1268Error read_registers(uint16_t address, uint8_t* buffer, size_t size);
	[[nodiscard]] SX1268Error calibrate_image(uint32_t freq);
	[[nodiscard]] SX1268Error set_dio_irq_params(uint16_t irq_mask, uint16_t dio_1_mask, uint16_t dio_2_mask, uint16_t dio_3_mask);
	[[nodiscard]] SX1268Error set_packet_params(uint16_t preamble_len,  RadioLoRaPacketLengthsMode_t header_type, uint8_t payload_len, RadioLoRaCrcModes_t crc_mode, RadioLoRaIQModes_t invert_iq);
	[[nodiscard]] SX1268Error set_packet(uint8_t* data, size_t len);
	[[nodiscard]] SX1268Error set_base_address(uint8_t tx_base, uint8_t rx_base);
	[[nodiscard]] SX1268Error set_pa_config(uint8_t pa_duty_cycle, uint8_t hp_max);
	[[nodiscard]] SX1268Error set_tx_params(int8_t power, RadioRampTimes_t ramp_time);
	[[nodiscard]] SX1268Error set_standby();
	[[nodiscard]] SX1268Error set_packet_type(RadioPacketType_t packet_type);
	[[nodiscard]] SX1268Error clear_irq();
	[[nodiscard]] SX1268Error check_device_errors();
	[[nodiscard]] SX1268Error check_device_state();

	SPIClass& spi;
	uint8_t pin_cs;
	uint8_t pin_busy;
	uint8_t pin_dio1;
	uint8_t pin_rxen;
	uint8_t pin_reset;
	uint8_t tx_power = 0;
	uint32_t frequency = 430000000;
	int prev_rssi;
	int prev_snr;
	int prev_signal_rssi;
	int prev_rx_error;
	bool busy_fault;
	SPISettings spiSettings = SPISettings(10000000, MSBFIRST, SPI_MODE0);
};