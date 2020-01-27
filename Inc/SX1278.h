/**
 * Author Wojciech Domski <Wojciech.Domski@gmail.com>
 * www: www.Domski.pl
 *
 * work based on DORJI.COM sample code and
 * https://github.com/realspinner/SX1278_LoRa
 */

#ifndef __SX1278_H__
#define __SX1278_H__

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#define SX1278_MAX_PACKET	256
#define SX1278_DEFAULT_TIMEOUT		3000

//Error Coding rate (CR)setting
#define SX1278_CR_4_5
//#define SX1278_CR_4_6
//#define SX1278_CR_4_7
//#define SX1278_CR_4_8
#ifdef   SX1278_CR_4_5
#define SX1278_CR	0x01
#else
#ifdef   SX1278_CR_4_6
#define SX1278_CR    0x02
#else
#ifdef   SX1278_CR_4_7
#define SX1278_CR    0x03
#else
#ifdef   SX1278_CR_4_8
#define SX1278_CR    0x04
#endif
#endif
#endif
#endif

//CRC Enable
#define SX1278_CRC_EN
#ifdef  SX1278_CRC_EN
#define SX1278_CRC   0x01
#else
#define SX1278_CRC   0x00
#endif
//RFM98 Internal registers Address
/********************LoRa mode***************************/
#define LR_RegFifo                                  0x00
// Common settings
#define LR_RegOpMode                                0x01
#define LR_RegFrfMsb                                 0x06
#define LR_RegFrfMid                                 0x07
#define LR_RegFrfLsb                                 0x08
// Tx settings
#define LR_RegPaConfig                              0x09
#define LR_RegPaRamp                                0x0A
#define LR_RegOcp                                   0x0B
#define LR_RegFrMsb                                 0x06
// Rx settings
#define LR_RegLna                                   0x0C
// LoRa registers
#define LR_RegFifoAddrPtr                           0x0D
#define LR_RegFifoTxBaseAddr                        0x0E
#define LR_RegFifoRxBaseAddr                        0x0F
#define LR_RegFifoRxCurrentaddr                     0x10
#define LR_RegIrqFlagsMask                          0x11
#define LR_RegIrqFlags                              0x12
#define LR_RegRxNbBytes                             0x13
#define LR_RegRxHeaderCntValueMsb                   0x14
#define LR_RegRxHeaderCntValueLsb                   0x15
#define LR_RegRxPacketCntValueMsb                   0x16
#define LR_RegRxPacketCntValueLsb                   0x17
#define LR_RegModemStat                             0x18
#define LR_RegPktSnrValue                           0x19
#define LR_RegPktRssiValue                          0x1A
#define LR_RegRssiValue                             0x1B
#define LR_RegHopChannel                            0x1C
#define LR_RegModemConfig1                          0x1D
#define LR_RegModemConfig2                          0x1E
#define LR_RegSymbTimeoutLsb                        0x1F
#define LR_RegPreambleMsb                           0x20
#define LR_RegPreambleLsb                           0x21
#define LR_RegPayloadLength                         0x22
#define LR_RegMaxPayloadLength                      0x23
#define LR_RegHopPeriod                             0x24
#define LR_RegFifoRxByteAddr                        0x25

// I/O settings
#define REG_LR_DIOMAPPING1                          0x40
#define REG_LR_DIOMAPPING2                          0x41

#define REG_DIO_MAPPING_1        					0x40
#define REG_LR_VERSION                              0x42
// Additional settings
#define REG_LR_PLLHOP                               0x44
#define REG_LR_TCXO                                 0x4B
#define REG_LR_PADAC                                0x4D
#define REG_LR_FORMERTEMP                           0x5B
#define REG_LR_AGCREF                               0x61
#define REG_LR_AGCTHRESH1                           0x62
#define REG_LR_AGCTHRESH2                           0x63
#define REG_LR_AGCTHRESH3                           0x64

#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_VERSION              0x42
#define REG_LNA                  0x0c
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_MODEM_CONFIG_3       0x26
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_INVERTIQ             0x33
#define REG_DETECTION_THRESHOLD  0x37
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_SYNC_WORD            0x39
/********************FSK/ook mode***************************/
#define  RegFIFO                0x00
#define  RegOpMode              0x01
#define  RegBitRateMsb      	0x02
#define  RegBitRateLsb      	0x03
#define  RegFdevMsb             0x04
#define  RegFdevLsb             0x05
#define  RegFreqMsb             0x06
#define  RegFreqMid             0x07
#define  RegFreqLsb         	0x08
#define  RegPaConfig            0x09
#define  RegPaRamp              0x0a
#define  RegOcp                 0x0b
#define  RegLna                 0x0c
#define  RegRxConfig            0x0d
#define  RegRssiConfig      	0x0e
#define  RegRssiCollision 		0x0f
#define  RegRssiThresh      	0x10
#define  RegRssiValue           0x11
#define  RegRxBw                0x12
#define  RegAfcBw               0x13
#define  RegOokPeak             0x14
#define  RegOokFix              0x15
#define  RegOokAvg              0x16
#define  RegAfcFei              0x1a
#define  RegAfcMsb              0x1b
#define  RegAfcLsb              0x1c
#define  RegFeiMsb              0x1d
#define  RegFeiLsb              0x1e
#define  RegPreambleDetect  	0x1f
#define  RegRxTimeout1      	0x20
#define  RegRxTimeout2      	0x21
#define  RegRxTimeout3      	0x22
#define  RegRxDelay             0x23
#define  RegOsc                 0x24
#define  RegPreambleMsb     	0x25
#define  RegPreambleLsb     	0x26
#define  RegSyncConfig      	0x27
#define  RegSyncValue1      	0x28
#define  RegSyncValue2      	0x29
#define  RegSyncValue3      	0x2a
#define  RegSyncValue4      	0x2b
#define  RegSyncValue5      	0x2c
#define  RegSyncValue6      	0x2d
#define  RegSyncValue7      	0x2e
#define  RegSyncValue8      	0x2f
#define  RegPacketConfig1       0x30
#define  RegPacketConfig2       0x31
#define  RegPayloadLength       0x32
#define  RegNodeAdrs            0x33
#define  RegInvertIQ			0x33
#define  RegBroadcastAdrs       0x34
#define  RegFifoThresh      	0x35
#define  RegSeqConfig1      	0x36
#define  RegSeqConfig2      	0x37
#define  RegTimerResol      	0x38
#define  RegTimer1Coef      	0x39
#define  RegTimer2Coef      	0x3a
#define  RegImageCal            0x3b
#define  RegInvertIQ2          0x3b
#define  RegTemp                0x3c
#define  RegLowBat              0x3d
#define  RegIrqFlags1           0x3e
#define  RegIrqFlags2           0x3f
#define  RegDioMapping1			0x40
#define  RegDioMapping2			0x41
#define  RegVersion				0x42
#define  RegPllHop				0x44
#define  RegPaDac				0x4d
#define  RegBitRateFrac			0x5d

#define  RegSyncWord  			0x39
/**********************************************************
 **Parameter table define
 **********************************************************/
#define SX1278_433MHZ			0

static const uint8_t SX1278_Frequency[1][3] = { { 0x6C, 0x80, 0x00 }, //434MHz
		};

#define SX1278_POWER_20DBM		0
#define SX1278_POWER_17DBM		1
#define SX1278_POWER_14DBM		2
#define SX1278_POWER_11DBM		3

static const uint8_t SX1278_Power[4] = { 0xFF, //20dbm
		0xFC, //17dbm
		0xF9, //14dbm
		0xF6, //11dbm
		};

#define SX1278_LORA_SF_6		0
#define SX1278_LORA_SF_7		1
#define SX1278_LORA_SF_8		2
#define SX1278_LORA_SF_9		3
#define SX1278_LORA_SF_10		4
#define SX1278_LORA_SF_11		5
#define SX1278_LORA_SF_12		6

static const uint8_t SX1278_SpreadFactor[7] = { 6, 7, 8, 9, 10, 11, 12 };

#define	SX1278_LORA_BW_7_8KHZ		0
#define	SX1278_LORA_BW_10_4KHZ		1
#define	SX1278_LORA_BW_15_6KHZ		2
#define	SX1278_LORA_BW_20_8KHZ		3
#define	SX1278_LORA_BW_31_2KHZ		4
#define	SX1278_LORA_BW_41_7KHZ		5
#define	SX1278_LORA_BW_62_5KHZ		6
#define	SX1278_LORA_BW_125KHZ		7
#define	SX1278_LORA_BW_250KHZ		8
#define	SX1278_LORA_BW_500KHZ		9

/*****************************************************************************/
#define PA_OUTPUT_RFO_PIN          0
#define PA_OUTPUT_PA_BOOST_PIN     1


#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

//SPISettings _spiSettings;
//SPIClass* _spi;
int _ss;
int _reset;
int _dio0;
long _frequency;
int _packetIndex;
int _implicitHeaderMode;
//void (*_onReceive)(int);




static const uint8_t SX1278_LoRaBandwidth[10] = { 0, //   7.8KHz,
		1, //  10.4KHz,
		2, //  15.6KHz,
		3, //  20.8KHz,
		4, //  31.2KHz,
		5, //  41.7KHz,
		6, //  62.5KHz,
		7, // 125.0KHz,
		8, // 250.0KHz,
		9  // 500.0KHz
		};

typedef enum _SX1278_STATUS {
	SLEEP, STANDBY, TX, RX
} SX1278_Status_t;

typedef struct {
	int pin;
	void * port;
} SX1278_hw_dio_t;

typedef struct {
	SX1278_hw_dio_t reset;
	SX1278_hw_dio_t dio0;
	SX1278_hw_dio_t nss;
	void * spi;
} SX1278_hw_t;

typedef struct {
	SX1278_hw_t * hw;

	long frequency;
	uint8_t power;
	uint8_t LoRa_Rate;
	uint8_t LoRa_BW;
	uint8_t packetLength;

	SX1278_Status_t status;

	uint8_t rxBuffer[SX1278_MAX_PACKET];
	uint8_t readBytes;
} SX1278_t;

//hardware
__weak void SX1278_hw_init(SX1278_hw_t * hw);
__weak void SX1278_hw_SetNSS(SX1278_hw_t * hw, int value);
__weak void SX1278_hw_Reset(SX1278_hw_t * hw);
__weak void SX1278_hw_SPICommand(SX1278_hw_t * hw, uint8_t cmd);
__weak uint8_t SX1278_hw_SPIReadByte(SX1278_hw_t * hw);
__weak void SX1278_hw_DelayMs(uint32_t msec);
__weak int SX1278_hw_GetDIO0(SX1278_hw_t * hw);

//logic

uint8_t SX1278_SPIRead(SX1278_t * module, uint8_t addr);
void SX1278_SPIWrite(SX1278_t * module, uint8_t addr, uint8_t cmd);
void SX1278_SPIBurstRead(SX1278_t * module, uint8_t addr, uint8_t *rxBuf,
		uint8_t length);
void SX1278_SPIBurstWrite(SX1278_t * module, uint8_t addr, uint8_t *txBuf,
		uint8_t length);
void SX1278_DIO0_InterruptHandler(SX1278_t * module);


void SX1278_config(SX1278_t * module, uint8_t frequency, uint8_t power,
		uint8_t LoRa_Rate, uint8_t LoRa_BW);
void SX1278_defaultConfig(SX1278_t * module);

void SX1278_entryLoRa(SX1278_t * module);
void SX1278_clearLoRaIrq(SX1278_t * module);
int SX1278_LoRaEntryRx(SX1278_t * module, uint8_t length, uint32_t timeout);
uint8_t SX1278_LoRaRxPacket(SX1278_t * module);
void SX1278_begin(SX1278_t * module, long frequency, uint8_t power,
		uint8_t LoRa_Rate, uint8_t LoRa_BW, uint8_t packetLength);
int SX1278_write_b(SX1278_t * module, uint8_t* txBuffer, uint8_t length,
		uint32_t timeout);
int SX1278_transmit(SX1278_t * module, uint8_t *txBuf, uint8_t length,
		uint32_t timeout);
int SX1278_(SX1278_t * module, uint8_t length, uint32_t timeoutT);
uint8_t SX1278_available(SX1278_t * module);
uint8_t SX1278_read(SX1278_t * module, uint8_t *rxBuf, uint8_t length);

uint8_t SX1278_RSSI_LoRa(SX1278_t * module);
uint8_t SX1278_RSSI(SX1278_t * module);

void SX1278_standby(SX1278_t * module);
void SX1278_sleep(SX1278_t * module);
void SX1278_entryLoRa(SX1278_t * module);
/*****************************************************/
void SX1278_setSyncWord(SX1278_t * module, int sw);
void LoRa_writeRegister(SX1278_t * module, uint8_t address, uint8_t value);
uint8_t LoRa_singleTransfer(SX1278_t * module, uint8_t address, uint8_t value);
uint8_t LoRa_readRegister(SX1278_t * module, uint8_t address);
//void LoRa_WR(SX1278_t * module, uint8_t address, uint8_t value);
//uint8_t LoRa_WRS(SX1278_t * module, uint8_t address, uint8_t value);
void LoRa_implicitHeaderMode(SX1278_t * module);
void LoRa_explicitHeaderMode(SX1278_t * module);
void LoRa_enableCrc(SX1278_t * module);
void LoRa_disableCrc(SX1278_t * module);
void LoRa_enableInvertIQ(SX1278_t * module);
void LoRa_disableInvertIQ(SX1278_t * module);
void LoRa_setOCP(SX1278_t * module, uint8_t mA);
void LoRa_setFrequency(SX1278_t * module, long frequency);
void LoRa_setTxPower(SX1278_t * module, int level, int outputPin);
int LoRa_beginPacket(SX1278_t * module, int implicitHeader);
size_t LoRa_write(SX1278_t * module, uint8_t byte);
size_t LoRa_write_b(SX1278_t * module, const uint8_t *buffer, size_t size);
int LoRa_endPacket(SX1278_t * module, bool async);
void LoRa_idle(SX1278_t * module);
bool LoRa_isTransmitting(SX1278_t * module);
int LoRa_begin(SX1278_t * module, long frequency);
void LoRa_sleep(SX1278_t * module);
void LoRa_setLdoFlag(SX1278_t * module);
int LoRa_getSpreadingFactor(SX1278_t * module);
void LoRa_setSpreadingFactor(SX1278_t * module, int sf);
long LoRa_getSignalBandwidth(SX1278_t * module);
void LoRa_setCodingRate4(SX1278_t * module, int denominator);
void LoRa_setPreambleLength(SX1278_t * module, long length);
void LoRa_setSyncWord(SX1278_t * module, int sw);
void LoRa_setSignalBandwidth(SX1278_t * module, long sbw);
void LoRa_init();

#endif
