#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_tim.h>
#include <stdlib.h>
struct servo{
	volatile unsigned int servo_1;
	volatile unsigned int servo_2;
	volatile unsigned int servo_3;
	volatile unsigned int servo_4;
	volatile unsigned int servo_5;
	volatile unsigned int servo_6;
	volatile unsigned int servo_7;
	volatile unsigned int servo_8;
	volatile unsigned int servo_9;
	volatile unsigned int servo_10;
	volatile unsigned int servo_11;
	volatile unsigned int servo_12;
	volatile unsigned int servo_13;
	volatile unsigned int servo_14;
	volatile unsigned int servo_15;
	volatile unsigned int servo_16;
};
extern struct servo servo;
void initializeTimers(void);
void initializePWMChannels(void);
void initializeGPIOs(void);
void initializeUSART(uint32_t);
void USART_puts(USART_TypeDef *, volatile char *);
void USART1_IRQHandler(void);
void delay(uint32_t);

