#include "initialize.h"
char dataFrame0;
char dataFrame1;
char dataFrame2;
char dataFrame3;
char dataFrame4;
int whoAmI=-1;
unsigned char adres = 0x01;	//Adres urzadzenia
volatile unsigned char BUFOR[5]; //Macierz zapisanej ramki
volatile unsigned char i = 0;
volatile int pozycja_BUFOR = 0;	// zmienna do zapisywania pod kolejne miejsca w bufoze
volatile int start_analiza_odbioru = 0;	// flaga do rozpoczêcia analizy odebranej ramki danych

void delay(uint32_t ms){
		ms *= 3360;
		while(ms--){
				__NOP();
		}
}

void initializeTimers(void){
	TIM_TimeBaseInitTypeDef timer2InitStructure;
	TIM_TimeBaseInitTypeDef timer3InitStructure;
	TIM_TimeBaseInitTypeDef timer4InitStructure;
	TIM_TimeBaseInitTypeDef timer5InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	timer2InitStructure.TIM_Prescaler = (uint16_t) ((SystemCoreClock) / 1000000) - 1;
		timer2InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
		timer2InitStructure.TIM_Period = 20000 - 1;
		timer2InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		timer2InitStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM2, &timer2InitStructure);
		TIM_Cmd(TIM2, ENABLE);
	
	timer3InitStructure.TIM_Prescaler = (uint16_t) ((SystemCoreClock) / 1000000) - 1;
		timer3InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
		timer3InitStructure.TIM_Period = 20000 - 1;
		timer3InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		timer3InitStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM3, &timer3InitStructure);
		TIM_Cmd(TIM3, ENABLE);
	
	timer4InitStructure.TIM_Prescaler = (uint16_t) ((SystemCoreClock) / 1000000) - 1;
		timer4InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
		timer4InitStructure.TIM_Period = 20000 - 1;
		timer4InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		timer4InitStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM4, &timer4InitStructure);
		TIM_Cmd(TIM4, ENABLE);
		
		timer5InitStructure.TIM_Prescaler = (uint16_t) ((SystemCoreClock) / 1000000) - 1;
		timer5InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
		timer5InitStructure.TIM_Period = 20000 - 1;
		timer5InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		timer5InitStructure.TIM_RepetitionCounter = 0;
		TIM_TimeBaseInit(TIM5, &timer5InitStructure);
		TIM_Cmd(TIM5, ENABLE);
}

void initializePWMChannels(void){
	TIM_OCInitTypeDef outputChannelInit = {0,};
	outputChannelInit.TIM_OCMode = TIM_OCMode_PWM1;
	outputChannelInit.TIM_Pulse = 1500;
	outputChannelInit.TIM_OutputState = TIM_OutputState_Enable;
	outputChannelInit.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM4, &outputChannelInit);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	
	TIM_OC2Init(TIM4, &outputChannelInit);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
	
	TIM_OC3Init(TIM4, &outputChannelInit);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
	
	TIM_OC4Init(TIM4, &outputChannelInit);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
	
	
	
	TIM_OC1Init(TIM2, &outputChannelInit);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_TIM2);
	
	TIM_OC2Init(TIM2, &outputChannelInit);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);
	
	TIM_OC3Init(TIM2, &outputChannelInit);
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_TIM2);
	
	TIM_OC4Init(TIM2, &outputChannelInit);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_TIM2);
	
	
	TIM_OC1Init(TIM3, &outputChannelInit);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
	
	TIM_OC2Init(TIM3, &outputChannelInit);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
	
	TIM_OC3Init(TIM3, &outputChannelInit);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	
	TIM_OC4Init(TIM3, &outputChannelInit);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
	
	
	
	TIM_OC1Init(TIM5, &outputChannelInit);
	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	
	TIM_OC2Init(TIM5, &outputChannelInit);
	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
	
	TIM_OC3Init(TIM5, &outputChannelInit);
	TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);
	
	TIM_OC4Init(TIM5, &outputChannelInit);
	TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM5);
}

void initializeGPIOs(void){
	GPIO_InitTypeDef gpioStructureTIM4;
	GPIO_InitTypeDef gpioStructureTIM2A;
	GPIO_InitTypeDef gpioStructurePinoutB;
	GPIO_InitTypeDef gpioStructurePinoutC;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		gpioStructureTIM4.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
		gpioStructureTIM4.GPIO_Mode = GPIO_Mode_AF;
		gpioStructureTIM4.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOD, &gpioStructureTIM4);
	
	gpioStructureTIM2A.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_5 | GPIO_Pin_6;
		gpioStructureTIM2A.GPIO_Mode = GPIO_Mode_AF;
		gpioStructureTIM2A.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &gpioStructureTIM2A);
	gpioStructurePinoutB.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_10 | GPIO_Pin_11;
		gpioStructurePinoutB.GPIO_Mode = GPIO_Mode_AF;
		gpioStructurePinoutB.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &gpioStructurePinoutB);
	gpioStructurePinoutC.GPIO_Pin = GPIO_Pin_7;
		gpioStructurePinoutC.GPIO_Mode = GPIO_Mode_AF;
		gpioStructurePinoutC.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOC, &gpioStructurePinoutC);
}

/* This funcion initializes the USART1 peripheral
 *
 * Arguments: baudrate --> the baudrate at which the USART is
 * 							 supposed to operate
 */
void initializeUSART(uint32_t baudrate){

	/* This is a concept that has to do with the libraries provided by ST
	 * to make development easier the have made up something similar to
	 * classes, called TypeDefs, which actually just define the common
	 * parameters that every peripheral needs to work correctly
	 *
	 * They make our life easier because we don't have to mess around with
	 * the low level stuff of setting bits in the correct registers
	 */
	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	/* enable APB2 peripheral clock for USART1
	 * note that only USART1 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* enable the peripheral clock for the pins used by
	 * USART1, PB6 for TX and PB7 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* This sequence sets up the TX and RX pins
	 * so they work correctly with the USART1 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	/* The RX and TX pins are now connected to their AF
	 * so that the USART1 can take over control of the
	 * pins
	 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); //
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

	/* Now the USART_InitStruct is used to define the
	 * properties of USART1
	 */
	USART_InitStruct.USART_BaudRate = baudrate;				// the baudrate is set to the value we passed into this init function
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;// we want the data frame size to be 8 bits (standard)
	USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
	USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
	USART_Init(USART1, &USART_InitStruct);					// again all the properties are passed to the USART_Init function which takes care of all the bit setting


	/* Here the USART1 receive interrupt is enabled
	 * and the interrupt controller is configured
	 * to jump to the USART1_IRQHandler() function
	 * if the USART1 receive interrupt occurs
	 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART1, ENABLE);
}

/* This function is used to transmit a string of characters via
 * the USART specified in USARTx.
 *
 * It takes two arguments: USARTx --> can be any of the USARTs e.g. USART1, USART2 etc.
 * 							 (volatile) char *s is the string you want to send
 *
 * Note: The string has to be passed to the function as a pointer because
 * 		 the compiler doesn't know the 'string' data type. In standard
 * 		 C a string is just an array of characters
 *
 * Note 2: At the moment it takes a volatile char because the received_string variable
 * 			 declared as volatile char --> otherwise the compiler will spit out warnings
 * */
void USART_puts(USART_TypeDef* USARTx, volatile char *s){

	while(*s){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) );
		USART_SendData(USARTx, *s);
		*s++;
	}
}

void USART1_IRQHandler(void){
	// check if the USART1 receive interrupt flag was set
	if( USART_GetITStatus(USART1, USART_IT_RXNE) ){

i = USART1->DR; // the character from the USART1 data register is saved in t

	if(i == 0x0A){
		if(pozycja_BUFOR == 4){		//Sprawdzenie czy koniec ramki na dobrej pozycji
			BUFOR[4] = 0x0A;
			start_analiza_odbioru = 1;
			pozycja_BUFOR = 0;	//Wroc na poczatek bufora
		}
		else{
			pozycja_BUFOR = 0;	//Wroc na poczatek bufora jezeli blad
		}
	}
	else{
		switch(pozycja_BUFOR){
			case 0:
			if(i == 0xFF){	//Sprawdz czy $FF
				BUFOR[0] = i;
				pozycja_BUFOR++;
			}
			else{
				pozycja_BUFOR = 0;	//Wroc na poczatek bufora jezeli blad
			}
			break;
			case 1:
			BUFOR[1] = i;	//Adres
			pozycja_BUFOR++;
			break;
			case 2:
			BUFOR[2] = i;	//Rozkaz
			dataFrame2=i;
					if(i==0xAF){
		whoAmI=1;
		}
			pozycja_BUFOR++;
			break;
			case 3:
			BUFOR[3] = i;	//Dane
			dataFrame3=i;
			pozycja_BUFOR++;
			break;
			default:
			pozycja_BUFOR = 0; //Jezeli blad odbioru
			break;

		}
	
	}
	}
}
