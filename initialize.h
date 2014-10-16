#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_tim.h>
#include <stdlib.h>
void initializeTimers(void);
void initializePWMChannels(void);
void initializeGPIOs(void);
void initializeUSART(uint32_t);
void USART_puts(USART_TypeDef *, volatile char *);
void USART1_IRQHandler(void);
