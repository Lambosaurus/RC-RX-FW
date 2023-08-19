
#include "Core.h"
#include "GPIO.h"
#include "UART.h"

#include "NRF24.h"


void MAIN_OnRx(const uint8_t * data, uint32_t size)
{
	__BKPT();
}

int main(void)
{
	CORE_Init();
	GPIO_EnableOutput(LED_PIN, GPIO_PIN_RESET);
	GPIO_EnableInput(BTN_PIN, GPIO_Pull_Up);
	UART_Init(UART_1, 115200, UART_Mode_Default);

	CORE_Delay(100);

	NRT24_Config_t conf = {
		.bitrate = NRF24_Bitrate_1MHz,
		.channel = 0,
		.power_dbm = 0,
		.retries = 0,
		.retry_delay_us = 500,
	};

	NRF24_Init(&conf);

	NRF24_Receive(MAIN_OnRx);

	while(1)
	{
		//*
		CORE_Delay(1000);
		uint8_t payload[8] = {
				0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07
		};
		GPIO_Write(LED_PIN, GPIO_PIN_SET);
		NRF24_Send(payload, sizeof(payload), false);
		GPIO_Write(LED_PIN, GPIO_PIN_RESET);
		/*/
		CORE_Idle();
		//*/
	}
}

