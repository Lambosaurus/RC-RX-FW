
#include "Core.h"
#include "GPIO.h"
#include "UART.h"

#include "NRF24.h"


int main(void)
{
	CORE_Init();
	GPIO_EnableOutput(LED_PIN, GPIO_PIN_RESET);
	GPIO_EnableInput(BTN_PIN, GPIO_Pull_Up);
	UART_Init(UART_1, 115200, UART_Mode_Default);

	CORE_Delay(100);
	NRF24_Init();

	while(1)
	{
		CORE_Delay(1000);
	}
}

