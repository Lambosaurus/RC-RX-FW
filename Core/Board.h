#ifndef BOARD_H
#define BOARD_H

//#define STM32L0
//#define STM32F0
#define STM32G0

// Core config
//#define CORE_USE_TICK_IRQ

// CLK config
//#define CLK_USE_HSE
//#define CLK_USE_LSE
//#define CLK_LSE_BYPASS
//#define CLK_LSE_FREQ		32768
#define CLK_SYSCLK_FREQ		32000000

// RTC config
//#define RTC_USE_IRQS

// US config
//#define US_TIM			TIM_22
//#define US_RES			1

// ADC config
//#define ADC_VREF	        3300

// GPIO config
#define GPIO_USE_IRQS
#define GPIO_IRQ4_ENABLE

// TIM config
//#define TIM_USE_IRQS
//#define TIM2_ENABLE

// UART config
#define UART1_PINS			(PB6 | PB7)
#define UART1_AF		 	GPIO_AF0_USART1
#define UART_BFR_SIZE     	64

// SPI config
#define SPI1_PINS		    (PA5 | PA6 | PA7)
#define SPI1_AF				GPIO_AF0_SPI1

// I2C config
//#define I2C1_GPIO			GPIOB
//#define I2C1_PINS			(GPIO_PIN_6 | GPIO_PIN_7)
//#define I2C1_AF			GPIO_AF1_I2C1
//#define I2C_USE_FASTMODEPLUS

// CAN config
//#define CAN_GPIO			GPIOB
//#define CAN_PINS			(GPIO_PIN_8 | GPIO_PIN_9)
//#define CAN_AF			GPIO_AF4_CAN
//#define CAN_DUAL_FIFO

// USB config
//#define USB_ENABLE
//#define USB_CLASS_CDC
//#define USB_CDC_BFR_SIZE	512

#define LED_PIN				PA2
#define BTN_PIN				PA1

#define NRF24_INT_PIN		PA4
#define NRF24_CS_PIN		PB0
#define NRF24_EN_PIN		PB1
#define NRF24_SPI			SPI_1


#endif /* BOARD_H */
