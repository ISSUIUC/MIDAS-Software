#include"E22.h"

#define DBG_PRINT(x) (void) 0
#define SX1268Check(x) if((x) == SX1268Error::BusyTimeout) { return SX1268Error::BusyTimeout; }
// #define DBG_PRINT(x) Serial.println(x);

SX1268Error SX1268::wait_on_busy(){
	int timeout = 1000;
	while (digitalRead(pin_busy) == HIGH)
	{
		delay(1);
		timeout -= 1;
		if (timeout < 0)
		{
			return SX1268Error::BusyTimeout;
		}
	}

    return SX1268Error::NoError;
}

SX1268Error SX1268::write_command(RadioCommands_t command, uint8_t* buffer, size_t size) {
    DBG_PRINT("write command start");
    SX1268Check(wait_on_busy());

    digitalWrite(pin_cs, LOW);
    spi.beginTransaction(spiSettings);
    spi.transfer((uint8_t)command);
    for(size_t i = 0; i < size; i++){
        spi.transfer(buffer[i]);
    }
    spi.endTransaction();
    digitalWrite(pin_cs, HIGH);
    DBG_PRINT("write command end");
    SX1268Check(wait_on_busy());

    return SX1268Error::NoError;
}

SX1268Error SX1268::read_command(RadioCommands_t command, uint8_t* buffer, size_t size) {
    DBG_PRINT("read command start");
    SX1268Check(wait_on_busy());

    digitalWrite(pin_cs, LOW);
    spi.beginTransaction(spiSettings);
    spi.transfer((uint8_t)command);
    spi.transfer(0x00);
    for(size_t i = 0; i < size; i++){
        buffer[i] = spi.transfer(0x00);
    }
    spi.endTransaction();
    digitalWrite(pin_cs, HIGH);
    DBG_PRINT("read command end");
    SX1268Check(wait_on_busy());

    return SX1268Error::NoError;
}

SX1268Error SX1268::write_buffer(uint8_t offest, const uint8_t* buffer, size_t size) {
    DBG_PRINT("write buffer start");
    SX1268Check(wait_on_busy());

    digitalWrite(pin_cs, LOW);
    spi.beginTransaction(spiSettings);
    spi.transfer(RADIO_WRITE_BUFFER);
    spi.transfer(offest);
    for(size_t i = 0; i < size; i++) {
        spi.transfer(buffer[i]);
    }
    spi.endTransaction();
    digitalWrite(pin_cs, HIGH);
    DBG_PRINT("write buffer end");
    SX1268Check(wait_on_busy());

    return SX1268Error::NoError;
}

SX1268Error SX1268::write_registers(uint16_t address, uint8_t* buffer, size_t size) {
    DBG_PRINT("write register start");
    SX1268Check(wait_on_busy());

    digitalWrite(pin_cs, LOW);
    spi.beginTransaction(spiSettings);
    spi.transfer(RADIO_WRITE_REGISTER);
    spi.transfer((address & 0xFF00) >> 8);
    spi.transfer(address & 0x00FF);

    for(size_t i = 0; i < size; i++){
        spi.transfer(buffer[i]);
    }

    spi.endTransaction();
    digitalWrite(pin_cs, HIGH);
    DBG_PRINT("write buffer end");
    SX1268Check(wait_on_busy());

    return SX1268Error::NoError;
}

SX1268Error SX1268::read_buffer(uint8_t offset, uint8_t* buffer, size_t size) {
    DBG_PRINT("read buffer start");
    SX1268Check(wait_on_busy());

    digitalWrite(pin_cs, LOW);
    spi.beginTransaction(spiSettings);
    spi.transfer(RADIO_READ_BUFFER);
    spi.transfer(offset); 
    spi.transfer(0x0);
    for(size_t i = 0; i < size; i++){
        buffer[i] = spi.transfer(0x0);
    }

    spi.endTransaction();
    digitalWrite(pin_cs, HIGH);
    DBG_PRINT("write buffer end");
    SX1268Check(wait_on_busy());

    return SX1268Error::NoError;
}

SX1268Error SX1268::read_registers(uint16_t address, uint8_t* buffer, size_t size) {
    DBG_PRINT("read registers start");
    SX1268Check(wait_on_busy());

    digitalWrite(pin_cs, LOW);
    spi.beginTransaction(spiSettings);
    spi.transfer(RADIO_READ_REGISTER);
    spi.transfer((address & 0xFF00) >> 8);
    spi.transfer(address & 0x00FF);
    spi.transfer(0x00);
    for(size_t i = 0; i < size; i++){
        buffer[i] = spi.transfer(0x00);
    }
    spi.endTransaction();
    digitalWrite(pin_cs, HIGH);
    DBG_PRINT("read registers end");
    SX1268Check(wait_on_busy());

    return SX1268Error::NoError;
}


SX1268Error SX1268::calibrate_image(uint32_t freq)
{
	uint8_t calFreq[2];

	if (freq > 900000000)
	{
		calFreq[0] = 0xE1;
		calFreq[1] = 0xE9;
	}
	else if (freq > 850000000)
	{
		calFreq[0] = 0xD7;
		calFreq[1] = 0xDB;
	}
	else if (freq > 770000000)
	{
		calFreq[0] = 0xC1;
		calFreq[1] = 0xC5;
	}
	else if (freq > 460000000)
	{
		calFreq[0] = 0x75;
		calFreq[1] = 0x81;
	}
	else if (freq > 425000000)
	{
		calFreq[0] = 0x6B;
		calFreq[1] = 0x6F;
	}
	SX1268Check(write_command(RADIO_CALIBRATEIMAGE, calFreq, 2));

    return SX1268Error::NoError;
}

SX1268Error SX1268::set_frequency(uint32_t frequency) {
    this->frequency = frequency;

    uint8_t buf[4];
	uint32_t freq = 0;
    SX1268Check(calibrate_image(frequency));
	freq = (uint32_t)((double)frequency / (double)FREQ_STEP);
	buf[0] = (uint8_t)((freq >> 24) & 0xFF);
	buf[1] = (uint8_t)((freq >> 16) & 0xFF);
	buf[2] = (uint8_t)((freq >> 8) & 0xFF);
	buf[3] = (uint8_t)(freq & 0xFF);
	SX1268Check(write_command(RADIO_SET_RFFREQUENCY, buf, 4));

    SX1268Check(check_device_errors());

    return SX1268Error::NoError;
}

SX1268Error SX1268::setup(){
    pinMode(pin_cs, OUTPUT);
    pinMode(pin_dio1, INPUT);
    pinMode(pin_busy, INPUT);
    pinMode(pin_rxen, OUTPUT);
    pinMode(pin_reset, OUTPUT);
    digitalWrite(pin_cs, HIGH);
    digitalWrite(pin_reset, LOW);
    delay(10);
    digitalWrite(pin_reset, HIGH);
    delay(1);
    DBG_PRINT("setup start");
    SX1268Check(wait_on_busy());
    SX1268Check(set_standby());
    SX1268Check(set_packet_type(PACKET_TYPE_LORA));
    SX1268Check(set_tx_power(22));
    SX1268Check(set_base_address(0x00, 0x00));
    SX1268Check(check_device_errors());

    return SX1268Error::NoError;
}

SX1268Error SX1268::set_dio_irq_params(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask)
{
	uint8_t buf[8];

	buf[0] = (uint8_t)((irqMask >> 8) & 0x00FF);
	buf[1] = (uint8_t)(irqMask & 0x00FF);
	buf[2] = (uint8_t)((dio1Mask >> 8) & 0x00FF);
	buf[3] = (uint8_t)(dio1Mask & 0x00FF);
	buf[4] = (uint8_t)((dio2Mask >> 8) & 0x00FF);
	buf[5] = (uint8_t)(dio2Mask & 0x00FF);
	buf[6] = (uint8_t)((dio3Mask >> 8) & 0x00FF);
	buf[7] = (uint8_t)(dio3Mask & 0x00FF);
	SX1268Check(write_command(RADIO_CFG_DIOIRQ, buf, 8));
    
    return SX1268Error::NoError;
}

SX1268Error SX1268::set_modulation_params(uint8_t spreading_factor, RadioLoRaBandwidths_t bandwidth, RadioLoRaCodingRates_t cr, bool low_data_rate) {
    uint8_t modulation_params[] = {
        spreading_factor,
        bandwidth,
        cr,
        low_data_rate,
    };
    SX1268Check(write_command(RADIO_SET_MODULATIONPARAMS, modulation_params, sizeof(modulation_params)));

    return SX1268Error::NoError;
}

SX1268Error SX1268::set_packet_params(uint16_t preamble_len,  RadioLoRaPacketLengthsMode_t header_type, uint8_t payload_len, RadioLoRaCrcModes_t crc_mode, RadioLoRaIQModes_t invert_iq) {
    uint8_t packet_params[] = {
        (uint8_t)(preamble_len >> 8),
        (uint8_t)preamble_len,
        header_type,
        payload_len,
        crc_mode,
        invert_iq
    };
    SX1268Check(write_command(RADIO_SET_PACKETPARAMS, packet_params, sizeof(packet_params)));

    return SX1268Error::NoError;
}

SX1268Error SX1268::set_packet(uint8_t* data, size_t len) {
    SX1268Check(write_buffer(0x00, data, len));

    return SX1268Error::NoError;
}

SX1268Error SX1268::set_base_address(uint8_t tx_base, uint8_t rx_base) {
    uint8_t baseaddr[] = {tx_base, rx_base};
    SX1268Check(write_command(RADIO_SET_BUFFERBASEADDRESS, baseaddr, sizeof(baseaddr)));

    return SX1268Error::NoError;
}

SX1268Error SX1268::set_pa_config(uint8_t pa_duty_cycle, uint8_t hp_max) {
    uint8_t pa_params[] = {
        pa_duty_cycle,
        hp_max,
        0x00, //Reserved,
        0x01, //Reserved,
    };

    SX1268Check(write_command(RADIO_SET_PACONFIG, pa_params, sizeof(pa_params)));

    return SX1268Error::NoError;
}

SX1268Error SX1268::set_tx_params(int8_t power, RadioRampTimes_t ramp_time) {
    SX1268Check(set_pa_config(0x04, 0x07));
    if(power > 22) {
        power = 22;
    } else if(power < -9) {
        power = -9;
    }

    uint8_t ocp_value = 0x38;
    SX1268Check(write_registers(REG_OCP, &ocp_value, sizeof(ocp_value)));

    uint8_t tx_params[] = {
        (uint8_t)power,
        ramp_time,
    };

    SX1268Check(write_command(RADIO_SET_TXPARAMS, tx_params, sizeof(tx_params)));

    return SX1268Error::NoError;
}

SX1268Error SX1268::set_standby() {
    uint8_t STANDBY_RC = 0;
    SX1268Check(write_command(RADIO_SET_STANDBY, &STANDBY_RC, sizeof(STANDBY_RC)));
    uint8_t readback{};
    SX1268Check(read_command(RADIO_GET_STATUS, &readback, sizeof(readback)));
    if(readback & 0x20 == 0) return SX1268Error::FailedReadback;


    return SX1268Error::NoError;
}

SX1268Error SX1268::set_packet_type(RadioPacketType_t packet_type) {
    uint8_t type = packet_type;
    SX1268Check(write_command(RADIO_SET_PACKETTYPE, &type, sizeof(type)));
    uint8_t readback{};
    SX1268Check(read_command(RADIO_GET_PACKETTYPE, &readback, sizeof(readback)));

    if(readback != type) {
        return SX1268Error::FailedReadback;
    }

    return SX1268Error::NoError;
}

SX1268Error SX1268::set_tx_power(int8_t dbm) {
    tx_power = dbm;
    SX1268Check(set_tx_params(dbm, RADIO_RAMP_800_US));


    return SX1268Error::NoError;
}

SX1268Error SX1268::clear_irq() {
    uint8_t clear_irq[] = {
        0xff,0xff //clear all
    };
    SX1268Check(write_command(RADIO_CLR_IRQSTATUS, clear_irq, sizeof(clear_irq)));

    return SX1268Error::NoError;
}


SX1268Error SX1268::recv(uint8_t* data, size_t len, size_t timeout_ms) {
    if(len > 255) return SX1268Error::BadParameter;
    SX1268Check(set_standby());
    SX1268Check(set_packet_type(PACKET_TYPE_LORA));
    SX1268Check(set_frequency(frequency));
    SX1268Check(set_base_address(0x00, 0x00));
    SX1268Check(set_packet_params(6, LORA_PACKET_EXPLICIT, len, LORA_CRC_ON, LORA_IQ_NORMAL));
    SX1268Check(set_dio_irq_params(
        IRQ_CRC_ERROR | IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_HEADER_ERROR,
        IRQ_CRC_ERROR | IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_HEADER_ERROR,
        IRQ_RADIO_NONE,
        IRQ_RADIO_NONE));

    SX1268Check(check_device_errors());
    size_t timeout = timeout_ms * TIME_DIVISION_PER_MS;
    uint8_t timeout_buf[] = {
        (uint8_t)((timeout >> 16) & 0xFF),
        (uint8_t)((timeout >> 8) & 0xFF),
        (uint8_t)((timeout >> 0) & 0xFF)
    };


    SX1268Check(write_command(RADIO_SET_RX, timeout_buf, sizeof(timeout)));
    for(int i = 0; i < timeout + 10; i++){
        if(digitalRead(pin_dio1)) {
            break;
        }
        delay(1);
        if(i == timeout + 9) {
            return SX1268Error::RxTimeout;
        }
    }
    uint8_t irq_buff[3]{};
    SX1268Check(read_command(RADIO_GET_IRQSTATUS, irq_buff, sizeof(irq_buff)));
    SX1268Check(clear_irq());
    uint16_t irq = irq_buff[1] + (irq_buff[2] << 8);
    // Serial.print("IRQ "); Serial.println(irq, 2);
    if(irq & IRQ_CRC_ERROR) {
        return SX1268Error::CrcError;
    } else if(irq & IRQ_RX_DONE) {
        uint8_t packet_info[3]{};
        SX1268Check(read_command(RADIO_GET_RXBUFFERSTATUS, packet_info, sizeof(packet_info)));

        uint8_t packet_len = packet_info[1];
        uint8_t packet_ptr = packet_info[2];
        uint8_t packet_data[4]{};
        SX1268Check(read_command(RADIO_GET_PACKETSTATUS, packet_data, sizeof(packet_data)));
        prev_rssi = -packet_data[1]/2;
        prev_snr = packet_data[2] / 4;
        prev_signal_rssi = -packet_data[3] / 2;
         // I think the datasheet is just lying
         // It's supposed to be packet_len then packet_ptr,
         // but it's not the case, so we just swap it
         if (packet_ptr < len) {
            len = packet_ptr;
         }
        SX1268Check(read_buffer(packet_len, data, len));
        return SX1268Error::NoError;
    } else if(irq & IRQ_RX_TX_TIMEOUT) {
        return SX1268Error::RxTimeout;
    } else {
        return SX1268Error::UnknownError;
    }
}

SX1268Error SX1268::send(uint8_t* data, size_t len) {
    if(len > 255) return SX1268Error::BadParameter;
    SX1268Check(set_standby());
    SX1268Check(set_packet_type(PACKET_TYPE_LORA));
    SX1268Check(set_frequency(frequency));
    SX1268Check(set_tx_power(tx_power));
    SX1268Check(set_base_address(0x00, 0x00));
    SX1268Check(set_packet(data, len));
    SX1268Check(set_packet_params(10, LORA_PACKET_EXPLICIT, len, LORA_CRC_ON, LORA_IQ_NORMAL));
    SX1268Check(set_dio_irq_params(
        IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
        IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
        IRQ_RADIO_NONE,
        IRQ_RADIO_NONE));
    SX1268Check(check_device_errors());
    uint8_t timeout[] = {0x00,0x00,0x00};
    SX1268Check(write_command(RADIO_SET_TX, timeout, sizeof(timeout)));
    for(int i = 0; i < 1000; i++){
        if(digitalRead(pin_dio1)) break;
        delay(1);
        if(i > 999) {
            Serial.println("TX TIMEOUT");
            return SX1268Error::TxTimeout;
        }
    }
    SX1268Check(clear_irq());

    return SX1268Error::NoError;
}

SX1268Error SX1268::check_device_errors() {
    uint8_t errors[2]{};
    SX1268Check(read_command(RADIO_GET_ERROR, errors, sizeof(errors)));
    if(errors[0] != 0 || errors[1] != 0) {
        uint8_t zero = 0;
        SX1268Check(write_command(RADIO_CLR_ERROR, &zero, sizeof(zero)));
        return SX1268Error::DeviceError;
    }

    return SX1268Error::NoError;
}

