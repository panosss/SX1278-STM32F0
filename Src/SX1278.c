#include "SX1278.h"
#include <string.h>
#include "gpio.h"
#include "spi.h"
#include <stdbool.h>

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40
#define IRQ_TX_DONE 			   0xF7
#define MAX_PKT_LENGTH             255

__weak void SX1278_hw_SetNSS(SX1278_hw_t * hw, int value) {
	HAL_GPIO_WritePin(hw->nss.port, hw->nss.pin,
			(value == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

__weak void SX1278_hw_SPICommand(SX1278_hw_t * hw, uint8_t cmd) {
	SX1278_hw_SetNSS(hw, 0);
	HAL_SPI_Transmit(hw->spi, &cmd, 1, 1000);
	while (HAL_SPI_GetState(hw->spi) != HAL_SPI_STATE_READY)
		;
}

__weak uint8_t SX1278_hw_SPIReadByte(SX1278_hw_t * hw) {
	uint8_t txByte = 0x00;
	uint8_t rxByte = 0x00;

	SX1278_hw_SetNSS(hw, 0);
	HAL_SPI_TransmitReceive(hw->spi, &txByte, &rxByte, 1, 1000);
	while (HAL_SPI_GetState(hw->spi) != HAL_SPI_STATE_READY)
		;
	return rxByte;
}





void LoRa_setTxPower(SX1278_t * module, int level, int outputPin) {
	if (PA_OUTPUT_RFO_PIN == outputPin) {
		// RFO
		if (level < 0) {
			level = 0;
		} else if (level > 14) {
			level = 14;
		}

		LoRa_writeRegister(module, LR_RegPaConfig, 0x70 | level);
	} else {
		// PA BOOST
		if (level > 17) {
			if (level > 20) {
				level = 20;
			}

			// subtract 3 from level, so 18 - 20 maps to 15 - 17
			level -= 3;

			// High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
			LoRa_writeRegister(module, RegPaDac, 0x87);
			LoRa_setOCP(module, 140);
		} else {
			if (level < 2) {
				level = 2;
			}
			//Default value PA_HF/LF or +17dBm
			LoRa_writeRegister(module, RegPaDac, 0x84);
			LoRa_setOCP(module, 100);
		}

		LoRa_writeRegister(module, LR_RegPaConfig, PA_BOOST | (level - 2));
	}
}


/*
 RegFrfMsb  (0x06)  7-0   Frf(23:16)  rw  0x6c  MSB of the RF carrier frequency
 RegFrfMid  (0x07)  7-0   Frf(15:8)   rw  0x80  MSB of the RF carrier frequency
 RegFrfLsb  (0x08)  7-0   Frf(7:0)    rw  0x00LSB of RF carrier frequency. Default value: 434.000 MHz.
 The RF frequency is taken into account internally only when:- entering FSRX/FSTX modes- re-starting the receiver
 */
void LoRa_setFrequency(SX1278_t * module, long frequency) {
	_frequency = frequency;

	 uint64_t frf = ((uint64_t) frequency << 19) / 32000000;

	 LoRa_writeRegister(module, LR_RegFrfMsb, (uint8_t) (frf >> 16));
	 LoRa_writeRegister(module, LR_RegFrfMid, (uint8_t) (frf >> 8));
	 LoRa_writeRegister(module, LR_RegFrfLsb, (uint8_t) (frf >> 0));
}



/**********************************************************************/

void LoRa_enableInvertIQ(SX1278_t * module) {
	LoRa_writeRegister(module, RegInvertIQ, 0x66);
	LoRa_writeRegister(module, RegInvertIQ2, 0x19);
}

void LoRa_disableInvertIQ(SX1278_t * module) {
	LoRa_writeRegister(module, RegInvertIQ, 0x27);
	LoRa_writeRegister(module, RegInvertIQ2, 0x1d);
}

void LoRa_setOCP(SX1278_t * module, uint8_t mA) {
	uint8_t ocpTrim = 27;

	if (mA <= 120) {
		ocpTrim = (mA - 45) / 5;
	} else if (mA <= 240) {
		ocpTrim = (mA + 30) / 10;
	}

	LoRa_writeRegister(module, LR_RegOcp, 0x20 | (0x1F & ocpTrim));
}

uint8_t LoRa_readRegister(SX1278_t * module, uint8_t address) {
	return LoRa_singleTransfer(module, address & 0x7f, 0x00); // 01111111. CLEAR bit 7, (wnr bit, 0 = for read access)
}

void LoRa_writeRegister(SX1278_t * module, uint8_t address, uint8_t value) {
	LoRa_singleTransfer(module, address | 0x80, value); // 0x80=10000000. SET bit 7(wnr bit, 1 = for write access). Leave 0 to 6 untouched.
}

void LoRa_explicitHeaderMode(SX1278_t * module) {
	_implicitHeaderMode = 0;

	LoRa_writeRegister(module, LR_RegModemConfig1,
			readRegister(LR_RegModemConfig1) & 0xfe);
}

void LoRa_implicitHeaderMode(SX1278_t * module) {
	_implicitHeaderMode = 1;

	LoRa_writeRegister(module, LR_RegModemConfig1,
			LoRa_readRegister(module, LR_RegModemConfig1) | 0x01);
}


/*
 SINGLE access: an address byte followed by a data byte is sent for a write access
 whereas an address byte is sent and a read byte is received for the read access.
 The NSS pin goes low at the beginning of the frame and goes high after the data byte.

 The first byte is the address byte.
 It is comprises:
 - wnr bit, which is 1 for write access and 0 for read access.
 - Then 7 bits of address, MSB first.

 The bitwise AND:  if both input bits are 1, the resulting output is 1, otherwise the output is 0.
 PORTD = PORTD & B00000011;  // CLEAR bits 2 - 7, leave pins PD0 and PD1 untouched
 address = address & 0x7f; // 0x7f=01111111= CLEAR bit 7(wnr bit, 0 = for read access), leave all others untouched

 The bitwise OR: The bitwise OR of two bits is 1 if either or both of the input bits is 1, otherwise it is 0.
 Used to set multiple bits in a bit-packed number.
 DDRD = DDRD | B11111100; // SET bits 2 to 7. Leave 0 and 1 untouched.
 address = address | 0x80; // 0x80=10000000 // SET bit 7(wnr bit, 1 = for write access). Leave 0 to 6 untouched.
 */
uint8_t LoRa_singleTransfer(SX1278_t * module, uint8_t address, uint8_t value) {
	uint8_t response = 0xFF;

	HAL_GPIO_WritePin(module->hw->nss.port, module->hw->nss.pin,
			GPIO_PIN_RESET);

	// read wnr bit value (0 = read access, 1 = write access)
	if (address >> 7 == 0) { // read
		SX1278_hw_SPICommand(module->hw, address);
		response = SX1278_hw_SPIReadByte(module->hw);
	} else { // write
		SX1278_hw_SPICommand(module->hw, address);
		SX1278_hw_SPICommand(module->hw, value);
	}

	HAL_GPIO_WritePin(module->hw->nss.port, module->hw->nss.pin, GPIO_PIN_SET);

	//if(address == 0b10000000){
	//printf("1.LoRa_singleTransfer RegFIFO= %s\r\n", LoRa_readRegister(module, RegFIFO));
	//}

	return response;
}

/*
 Returns 1 if the module entered transmission mode successfully.
 Returns 0 if a timeout period was reached.
 */
int LoRa_beginPacket(SX1278_t * module, int implicitHeader) {
	if (LoRa_isTransmitting(module)) {
		return 0;
	}

	// put in standby mode (FIFO data buffer (RegFIFO) is cleared)
	LoRa_idle(module);

	if (implicitHeader) {
		_implicitHeaderMode = 1;

		LoRa_writeRegister(module, LR_RegModemConfig1,
				LoRa_readRegister(module, LR_RegModemConfig1) | 0x01);

	} else {
		_implicitHeaderMode = 0;

		LoRa_writeRegister(module, LR_RegModemConfig1,
				LoRa_readRegister(module, LR_RegModemConfig1) & 0xfe);
	}

	// reset FIFO address and payload length
	LoRa_writeRegister(module, LR_RegFifoAddrPtr, 0);
	LoRa_writeRegister(module, LR_RegPayloadLength, 0);

	return 1;
}


size_t LoRa_write(SX1278_t * module, uint8_t byte) {
	//printf("1.LoRa_write byte= %i\r\n", byte);
	return LoRa_write_b(module, &byte, sizeof(byte));
}

/*
 FIFO access: if the address byte corresponds to the address of the FIFO,
 then succeeding data byte will address the FIFO.
 The address is not automatically incremented but is memorized and does not need to be sent between each data byte.
 The NSS pin goes low at the beginning of the frame and stay low between each byte.
 It goes high only after the last byte transfer
 */
size_t LoRa_write_b(SX1278_t * module, const uint8_t *buffer, size_t size) {
	int currentLength = LoRa_readRegister(module, LR_RegPayloadLength);

	//printf("1.LoRa_write_b buffer= %s\r\n", buffer);

	// check size
	if ((currentLength + size) > MAX_PKT_LENGTH) {
		size = MAX_PKT_LENGTH - currentLength;
	}

	// write data  // Write Data FIFO :LINE3
	for (size_t i = 0; i < size; i++) {
		LoRa_writeRegister(module, RegFIFO, buffer[i]);	//buffer[i]

		//printf("3.LoRa_write_b RegFIFO[%i]= %s\r\n", i, LoRa_readRegister(module, RegFIFO));
	}



	// update length  // 2. Write PayloadLength bytes to the FIFO (RegFifo) ????????
	LoRa_writeRegister(module, LR_RegPayloadLength, size);
	//printf("4.LoRa_write_b currentLength+size=%u  PlL= %u\r\n", currentLength + size, LoRa_readRegister(module, LR_RegPayloadLength));

	//LoRa_writeRegister(module, LR_RegOpMode, 0x8b); //Tx Mode  10001011   2-0:Mode (011->Transmitter mode (Tx))  3:LowFrequencyModeOn:1->Low Frequency Mode   6-5:ModulationType:00->FSK  7:LongRangeMode:1=LoRaTM Mode

	return size;
}

int LoRa_endPacket(SX1278_t * module, bool async) {

	// put in TX mode  // Mode Request TX
	LoRa_writeRegister(module, RegOpMode, MODE_LONG_RANGE_MODE | MODE_TX);

	if (async){
		// grace time is required for the radio
		HAL_Delay(150);
	}
	else {
		// wait for IRQ TxDone
		while ((LoRa_readRegister(module, REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK)
				== 0) {
			//yield();
		}

		// clear IRQ's
		LoRa_writeRegister(module, REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
		//SPIWrite(LR_RegIrqFlags,0xFF);
	}

	// Enter Standby mode
	//LoRa_idle(module);

	return 1;
}

void LoRa_idle(SX1278_t * module) {
	LoRa_writeRegister(module, RegOpMode, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

bool LoRa_isTransmitting(SX1278_t * module) {
	if ((LoRa_readRegister(module, RegOpMode) & MODE_TX) == MODE_TX) {
		return true;
	}

	if (LoRa_readRegister(module, LR_RegIrqFlags) & IRQ_TX_DONE_MASK) {
		// clear IRQ's
		LoRa_writeRegister(module, LR_RegIrqFlags, IRQ_TX_DONE_MASK);
	}

	return false;
}

void LoRa_init(){

}




int LoRa_begin(SX1278_t * module, long frequency) {

	// set SS high
	HAL_GPIO_WritePin(module->hw->nss.port, module->hw->nss.pin, GPIO_PIN_SET);

	// perform reset
	HAL_GPIO_WritePin(module->hw->reset.port, module->hw->reset.pin,
			GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(module->hw->reset.port, module->hw->reset.pin,
			GPIO_PIN_SET);
	HAL_Delay(10);

	// check version
	uint8_t version = LoRa_readRegister(module, REG_VERSION);
	if (version != 0x12) {
		return 0;
	}

	// put in sleep mode. It 's necessary for changing the frequency
	LoRa_sleep(module);

	// set frequency
	LoRa_setFrequency(module, frequency);

	// set base addresses
	LoRa_writeRegister(module, REG_FIFO_TX_BASE_ADDR, 0);
	LoRa_writeRegister(module, REG_FIFO_RX_BASE_ADDR, 0);

	// set LNA boost
	LoRa_writeRegister(module, REG_LNA,
			LoRa_readRegister(module, REG_LNA) | 0x03);

	// set auto AGC
	LoRa_writeRegister(module, REG_MODEM_CONFIG_3, 0x04);

	// set output power to 17 dBm
	//printf("1.LoRa_begin power= %i\r\n", SX1278_Power[module->power]);
	LoRa_setTxPower(module, 17, PA_OUTPUT_PA_BOOST_PIN);

	// put in standby mode
	LoRa_idle(module);

	return 1;
}



void LoRa_sleep(SX1278_t * module) {
	module->status = SLEEP;
	LoRa_writeRegister(module, RegOpMode, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

int LoRa_getSpreadingFactor(SX1278_t * module) {
	return LoRa_readRegister(module, REG_MODEM_CONFIG_2) >> 4;
}

void LoRa_setSpreadingFactor(SX1278_t * module, int sf) {
	if (sf < 6) {
		sf = 6;
	} else if (sf > 12) {
		sf = 12;
	}

	if (sf == 6) {
		LoRa_writeRegister(module, REG_DETECTION_OPTIMIZE, 0xc5);
		LoRa_writeRegister(module, REG_DETECTION_THRESHOLD, 0x0c);
	} else {
		LoRa_writeRegister(module, REG_DETECTION_OPTIMIZE, 0xc3);
		LoRa_writeRegister(module, REG_DETECTION_THRESHOLD, 0x0a);
	}

	LoRa_writeRegister(module, REG_MODEM_CONFIG_2,
			(LoRa_readRegister(module, REG_MODEM_CONFIG_2) & 0x0f)
					| ((sf << 4) & 0xf0));
	LoRa_setLdoFlag(module);
}

void LoRa_setSignalBandwidth(SX1278_t * module, long sbw) {
	int bw;

	if (sbw <= 7.8E3) {
		bw = 0;
	} else if (sbw <= 10.4E3) {
		bw = 1;
	} else if (sbw <= 15.6E3) {
		bw = 2;
	} else if (sbw <= 20.8E3) {
		bw = 3;
	} else if (sbw <= 31.25E3) {
		bw = 4;
	} else if (sbw <= 41.7E3) {
		bw = 5;
	} else if (sbw <= 62.5E3) {
		bw = 6;
	} else if (sbw <= 125E3) {
		bw = 7;
	} else if (sbw <= 250E3) {
		bw = 8;
	} else /*if (sbw <= 250E3)*/{
		bw = 9;
	}

	LoRa_writeRegister(module, REG_MODEM_CONFIG_1,
			(LoRa_readRegister(module, REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
	LoRa_setLdoFlag(module);
}

void LoRa_setLdoFlag(SX1278_t * module) {
	// Section 4.1.1.5
	long symbolDuration = 1000
			/ (LoRa_getSignalBandwidth(module)
					/ (1L << LoRa_getSpreadingFactor(module)));

	// Section 4.1.1.6
	bool ldoOn = symbolDuration > 16;

	uint8_t config3 = LoRa_readRegister(module, REG_MODEM_CONFIG_3);
	bitWrite(config3, 3, ldoOn);
	LoRa_writeRegister(module, REG_MODEM_CONFIG_3, config3);
}

long LoRa_getSignalBandwidth(SX1278_t * module) {
	int bw = (LoRa_readRegister(module, REG_MODEM_CONFIG_1) >> 4);

	switch (bw) {
	case 0:
		return 7.8E3;
	case 1:
		return 10.4E3;
	case 2:
		return 15.6E3;
	case 3:
		return 20.8E3;
	case 4:
		return 31.25E3;
	case 5:
		return 41.7E3;
	case 6:
		return 62.5E3;
	case 7:
		return 125E3;
	case 8:
		return 250E3;
	case 9:
		return 500E3;
	}

	return -1;
}

void LoRa_setCodingRate4(SX1278_t * module, int denominator) {
	if (denominator < 5) {
		denominator = 5;
	} else if (denominator > 8) {
		denominator = 8;
	}

	int cr = denominator - 4;

	LoRa_writeRegister(module, REG_MODEM_CONFIG_1,
			(LoRa_readRegister(module, REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void LoRa_setPreambleLength(SX1278_t * module, long length) {
	LoRa_writeRegister(module, REG_PREAMBLE_MSB, (uint8_t) (length >> 8));
	LoRa_writeRegister(module, REG_PREAMBLE_LSB, (uint8_t) (length >> 0));
}

void LoRa_setSyncWord(SX1278_t * module, int sw) {
	LoRa_writeRegister(module, REG_SYNC_WORD, sw);
}
