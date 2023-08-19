#ifndef NRF24_H
#define NRF24_H

#include "STM32X.h"

/*
 * PUBLIC DEFINITIONS
 */

#define NRF24_PAYLOAD_MAX	32

/*
 * PUBLIC TYPES
 */

typedef enum {
	NRF24_Bitrate_250KHz 	= 0x20,
	NRF24_Bitrate_1MHz 		= 0x00,
	NRF24_Bitrate_2MHz 		= 0x08,
} NRF24_Bitrate_t;

typedef struct {
	uint8_t channel;
	uint8_t retries;
	uint16_t retry_delay_us; // us
	int8_t power_dbm;
	NRF24_Bitrate_t bitrate;
} NRT24_Config_t;

typedef void(*NRF24_RxCallback_t)(const uint8_t * data, uint32_t size);

/*
 * PUBLIC FUNCTIONS
 */

void NRF24_Init(const NRT24_Config_t * config);

void NRF24_SetChannel(uint8_t channel);
void NRF24_Send(const uint8_t * data, uint32_t size, bool ack);

// Once NRF24_Receive is started, NRF24_StopReceive must be called before any other transactions are made.
void NRF24_Receive(NRF24_RxCallback_t callback);
void NRF24_StopReceive(void);

/*
 * EXTERN DECLARATIONS
 */

#endif //NRF24_H
