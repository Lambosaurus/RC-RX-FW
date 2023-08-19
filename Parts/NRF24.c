
#include "NRF24.h"
#include "SPI.h"
#include "GPIO.h"
#include "US.h"

#include <stdio.h>

/*
 * PRIVATE DEFINITIONS
 */

#define NRF24_CMD_R_REGISTER			0x00
#define NRF24_CMD_W_REGISTER			0x20
#define NRF24_CMD_R_RX_PL_WID			0x60
#define NRF24_CMD_R_RX_PAYLOAD			0x61
#define NRF24_CMD_FLUSH_RX				0xE2
#define NRF24_CMD_W_TX_PAYLOAD			0xA0
#define NRF24_CMD_FLUSH_TX				0xE1
#define NRF24_CMD_REUSE_TX_PL			0xE3
#define NRF24_CMD_W_ACK_PAYLOAD			0xA1
#define NRF24_CMD_W_TX_PAYLOAD_NOACK	0xB0
#define NRF24_CMD_NOP					0xFF

#define NRF24_REG_CONFIG				0x00
#define NRF24_REG_EN_AA					0x01
#define NRF24_REG_EN_RXADDR				0x02
#define NRF24_REG_SETUP_AW				0x03
#define NRF24_REG_SETUP_RETR			0x04
#define NRF24_REG_RF_CH					0x05
#define NRF24_REG_RF_SETUP				0x06
#define NRF24_REG_STATUS				0x07
#define NRF24_REG_OBSERVE_TX			0x08
#define NRF24_REG_RPD					0x09
#define NRF24_REG_RX_ADDR_PX			0x0A
#define NRF24_REG_TX_ADDR				0x10
#define NRF24_REG_RX_PW_PX				0x11
#define NRF24_REG_FIFO_STATUS			0x17
#define NRF24_REG_DYNPD					0x1C
#define NRF24_REG_FEATURE				0x1D

#define NRF24_CONFIG_MASK_RX_DR			0x40
#define NRF24_CONFIG_MASK_TX_DS			0x20
#define NRF24_CONFIG_MASK_MAX_RT		0x10
#define NRF24_CONFIG_EN_CRC				0x08
#define NRF24_CONFIG_CRCO				0x04
#define NRF24_CONFIG_PWR_UP				0x02
#define NRF24_CONFIG_PRIM_RX			0x01

#define NRF24_FEATURE_EN_DPL			0x04
#define NRF24_FEATURE_EN_ACK_PL			0x02
#define NRF24_FEATURE_EN_DYN_ACK		0x01

// The RX pipes we are enabling.
#define NRF24_PIPES						0x01


#define NRF24_IRQ_NONE					(NRF24_CONFIG_MASK_MAX_RT | NRF24_CONFIG_MASK_TX_DS | NRF24_CONFIG_MASK_RX_DR)
#define NRF24_IRQ_RX					(NRF24_CONFIG_MASK_MAX_RT | NRF24_CONFIG_MASK_TX_DS)
#define NRF24_IRQ_TX					(NRF24_CONFIG_MASK_RX_DR)

/*
 * PRIVATE TYPES
 */

/*
 * PRIVATE PROTOTYPES
 */

static uint8_t NRF24_ReadBytes(uint8_t cmd, uint8_t * rx, uint32_t size);
static uint8_t NRF24_WriteBytes(uint8_t cmd, const uint8_t * tx, uint32_t size);
static uint8_t NRF24_ReadStatus(void);
static uint8_t NRF24_ReadReg(uint8_t reg);
static void NRF24_WriteReg(uint8_t reg, uint8_t value);

static void NRF24_EnterTx(void);
static void NRF24_EnterRx(void);
static void NRF24_Sleep(void);

static void NRF24_SetPacketLength(uint8_t length);
static void NRF24_SetRfConf(NRF24_Bitrate_t bitrate, int8_t power_dbm);
static void NRF24_ConfigAutoAck(uint8_t retries, uint16_t retry_delay);
static void NRF24_FlushPackets(void);

static void NRF24_ISR(void);

/*
 * PRIVATE VARIABLES
 */

static struct {
	NRF24_RxCallback_t on_rx;
} gNRF24;

/*
 * PUBLIC FUNCTIONS
 */

void NRF24_Init(const NRT24_Config_t * config)
{
	GPIO_EnableOutput(NRF24_CS_PIN, GPIO_PIN_SET);
	GPIO_EnableOutput(NRF24_EN_PIN, GPIO_PIN_RESET);
	SPI_Init(NRF24_SPI, 1000000, SPI_Mode_0);

	NRF24_Sleep();

	NRF24_ConfigAutoAck(config->retries, config->retry_delay_us);
	NRF24_SetChannel(config->channel);
	NRF24_SetRfConf(config->bitrate, config->power_dbm);
	NRF24_SetPacketLength(0); // Dynamic
	NRF24_FlushPackets();

	GPIO_EnableInput(NRF24_INT_PIN, GPIO_Pull_Up);
	GPIO_OnChange(NRF24_INT_PIN, GPIO_IT_Falling, NRF24_ISR);
}

void NRF24_SetAddress(uint8_t * rx_addr, uint8_t * tx_addr, uint32_t size)
{
	// Only using pipe 0. See NRF24_PIPES.
	NRF24_WriteReg(NRF24_REG_SETUP_AW, size - 2);
	NRF24_WriteBytes(NRF24_CMD_W_REGISTER | NRF24_REG_RX_ADDR_PX, rx_addr, size);
	NRF24_WriteBytes(NRF24_CMD_W_REGISTER | NRF24_REG_TX_ADDR, tx_addr, size);
}

void NRF24_SetChannel(uint8_t channel)
{
	NRF24_WriteReg(NRF24_REG_RF_CH, channel);
}

void NRF24_Send(const uint8_t * data, uint32_t size, bool ack)
{
	uint8_t command = ack ? NRF24_CMD_W_TX_PAYLOAD : NRF24_CMD_W_TX_PAYLOAD_NOACK;
	uint8_t status = NRF24_WriteBytes(command, data, size);

	NRF24_EnterTx();

	// Strobe the CS pin to request packet send.
	GPIO_Write(NRF24_EN_PIN, GPIO_PIN_SET);
	US_Delay(10);
	GPIO_Write(NRF24_EN_PIN, GPIO_PIN_RESET);
}

void NRF24_Receive(NRF24_RxCallback_t callback)
{
	gNRF24.on_rx = callback;
	NRF24_EnterRx();
}

void NRF24_StopReceive(void)
{
	gNRF24.on_rx = NULL;
	NRF24_Sleep();
}

/*
 * PRIVATE FUNCTIONS
 */

static void NRF24_FlushPackets(void)
{
	NRF24_WriteBytes(NRF24_CMD_FLUSH_RX, NULL, 0);
	NRF24_WriteBytes(NRF24_CMD_FLUSH_TX, NULL, 0);
}

static void NRF24_SetPacketLength(uint8_t length)
{
	// Length = 0 indicates dynamic payload length

	uint8_t feature = length ? NRF24_FEATURE_EN_DPL : 0;
	NRF24_WriteReg(NRF24_REG_FEATURE, feature);

	uint8_t dynpd = length ? NRF24_PIPES : 0;
	NRF24_WriteReg(NRF24_REG_DYNPD, dynpd);

	if (length)
	{
		NRF24_WriteReg(NRF24_REG_RX_PW_PX, length);
	}
}

static void NRF24_SetRfConf(NRF24_Bitrate_t bitrate, int8_t power_dbm)
{
	power_dbm = (power_dbm > 0) ? 0 : (power_dbm < -18) ? -18 : power_dbm;
	uint8_t power = (power_dbm + 18) / 6;
	uint8_t rf = bitrate | power;
	NRF24_WriteReg(NRF24_REG_RF_SETUP, rf);
}

static void NRF24_ConfigAutoAck(uint8_t retries, uint16_t retry_delay)
{
	NRF24_WriteReg(NRF24_REG_EN_AA, retries ? NRF24_PIPES : 0x00);

	uint8_t ard = retry_delay / 250;
	ard = ard > 0 ? ard - 1 : 0;
	uint8_t arc = retries > 0xF ? 0xF : retries;

	NRF24_WriteReg(NRF24_REG_SETUP_RETR, (ard << 4) | arc);
}

static void NRF24_EnterTx(void)
{
	GPIO_Write(NRF24_EN_PIN, GPIO_PIN_RESET);
	uint8_t cfg = NRF24_CONFIG_PWR_UP | NRF24_IRQ_RX;
	NRF24_WriteReg(NRF24_REG_CONFIG, cfg);
}

static void NRF24_EnterRx(void)
{
	uint8_t cfg = NRF24_CONFIG_PWR_UP | NRF24_CONFIG_PRIM_RX | NRF24_IRQ_RX;
	NRF24_WriteReg(NRF24_REG_CONFIG, cfg);
	// The EN pin now controls the receiver
	GPIO_Write(NRF24_EN_PIN, GPIO_PIN_SET);
}

static void NRF24_Sleep(void)
{
	GPIO_Write(NRF24_EN_PIN, GPIO_PIN_RESET);
	uint8_t cfg = NRF24_IRQ_NONE;
	NRF24_WriteReg(NRF24_REG_CONFIG, cfg);
}

static uint8_t NRF24_ReadBytes(uint8_t cmd, uint8_t * rx, uint32_t size)
{
	GPIO_Write(NRF24_CS_PIN, GPIO_PIN_RESET);
	US_Delay(1);
	uint8_t status = SPI_TransferByte(NRF24_SPI, cmd);
	SPI_Read(NRF24_SPI, rx, size);
	GPIO_Write(NRF24_CS_PIN, GPIO_PIN_SET);
	return status;
}

static uint8_t NRF24_WriteBytes(uint8_t cmd, const uint8_t * tx, uint32_t size)
{
	GPIO_Write(NRF24_CS_PIN, GPIO_PIN_RESET);
	US_Delay(1);
	uint8_t status = SPI_TransferByte(NRF24_SPI, cmd);
	SPI_Write(NRF24_SPI, tx, size);
	GPIO_Write(NRF24_CS_PIN, GPIO_PIN_SET);
	return status;
}

static uint8_t NRF24_ReadStatus(void)
{
	GPIO_Write(NRF24_CS_PIN, GPIO_PIN_RESET);
	US_Delay(1);
	uint8_t status = SPI_TransferByte(NRF24_SPI, NRF24_CMD_NOP);
	GPIO_Write(NRF24_CS_PIN, GPIO_PIN_SET);
	return status;
}

static uint8_t NRF24_ReadReg(uint8_t reg)
{
	uint8_t value;
	NRF24_ReadBytes(NRF24_CMD_R_REGISTER | reg, &value, 1);
	return value;
}

static void NRF24_WriteReg(uint8_t reg, uint8_t value)
{
	NRF24_WriteBytes(NRF24_CMD_W_REGISTER | reg, &value, 1);
}

/*
 * INTERRUPT ROUTINES
 */


static void NRF24_ISR(void)
{
	if (gNRF24.on_rx)
	{
		// Check we are in recive mode at all.
		uint8_t size;
		NRF24_ReadBytes(NRF24_CMD_R_RX_PL_WID, &size, sizeof(size));

		// Truncate the size in case of corruption
		size &= (NRF24_PAYLOAD_MAX - 1);

		uint8_t payload[NRF24_PAYLOAD_MAX];
		NRF24_ReadBytes(NRF24_CMD_R_RX_PAYLOAD, payload, size);

		gNRF24.on_rx(payload, size);
	}
}

