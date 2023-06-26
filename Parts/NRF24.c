
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

/*
 * PRIVATE TYPES
 */

/*
 * PRIVATE PROTOTYPES
 */

static uint8_t NRF24_ReadStatus(void);
static uint8_t NRF24_ReadReg(uint8_t reg);
static void NRF24_WriteReg(uint8_t reg, uint8_t value);
static void NRF24_ForceVar(void * var);

/*
 * PRIVATE VARIABLES
 */

/*
 * PUBLIC FUNCTIONS
 */

void NRF24_Init(void)
{
	GPIO_EnableOutput(NRF24_CS_PIN, GPIO_PIN_SET);
	GPIO_EnableOutput(NRF24_EN_PIN, GPIO_PIN_RESET);
	SPI_Init(NRF24_SPI, 1000000, SPI_Mode_0);

	/*
	NRF24_WriteReg(NRF24_REG_CONFIG, 0x08);

	uint8_t regs[11];
	for (uint32_t i = 0; i < sizeof(regs); i++)
	{
		regs[i] = NRF24_ReadReg(i);
	}
	*/

	uint8_t status = NRF24_ReadStatus();
	__BKPT();
	NRF24_ForceVar(&status);
}

/*
 * PRIVATE FUNCTIONS
 */

static void NRF24_ForceVar(void * var)
{
	int v = *(uint8_t*)var;
    char bfr[16];
	snprintf(bfr, sizeof(bfr), "%d", v);
}

static uint8_t NRF24_Read(uint8_t cmd, uint8_t * rx, uint32_t size)
{
	GPIO_Write(NRF24_CS_PIN, GPIO_PIN_RESET);
	US_Delay(1);
	uint8_t status = SPI_TransferByte(NRF24_SPI, cmd);
	SPI_Read(NRF24_SPI, rx, size);
	GPIO_Write(NRF24_CS_PIN, GPIO_PIN_SET);
	return status;
}

static uint8_t NRF24_Write(uint8_t cmd, const uint8_t * tx, uint32_t size)
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
	NRF24_Read(NRF24_CMD_R_REGISTER | reg, &value, 1);
	return value;
}

static void NRF24_WriteReg(uint8_t reg, uint8_t value)
{
	NRF24_Write(NRF24_CMD_W_REGISTER | reg, &value, 1);
}

/*
 * INTERRUPT ROUTINES
 */

