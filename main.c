#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_tim.h>

char dataFrame0;
char dataFrame1;
char dataFrame2;
char dataFrame3;
char dataFrame4;
unsigned char adres = 0x01;	//Adres urzadzenia

volatile unsigned char BUFOR[5]; //Macierz zapisanej ramki
volatile unsigned char i = 0;
volatile int pozycja_BUFOR = 0;	// zmienna do zapisywania pod kolejne miejsca w bufoze
volatile int start_analiza_odbioru = 0;	// flaga do rozpoczęcia analizy odebranej ramki danych

void InitializeTimer()
{
	TIM_TimeBaseInitTypeDef timerInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    //timerInitStructure.TIM_Prescaler = (uint16_t) (((SystemCoreClock / 3360000) / 2));
	timerInitStructure.TIM_Prescaler = (uint16_t) ((SystemCoreClock) / 1000000) - 1;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = 20000 - 1;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &timerInitStructure);
    TIM_Cmd(TIM4, ENABLE);
}

void InitializePWMChannel()
{
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
}

/* This funcion initializes the USART1 peripheral
 *
 * Arguments: baudrate --> the baudrate at which the USART is
 * 						   supposed to operate
 */
void init_USART1(uint32_t baudrate){

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
 * 						   (volatile) char *s is the string you want to send
 *
 * Note: The string has to be passed to the function as a pointer because
 * 		 the compiler doesn't know the 'string' data type. In standard
 * 		 C a string is just an array of characters
 *
 * Note 2: At the moment it takes a volatile char because the received_string variable
 * 		   declared as volatile char --> otherwise the compiler will spit out warnings
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
		if(pozycja_BUFOR == 4){	  //Sprawdzenie czy koniec ramki na dobrej pozycji
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
			if(i == 0xFF){  //Sprawdz czy $FF
				BUFOR[0] = i;
				pozycja_BUFOR++;
			}
			else{
				pozycja_BUFOR = 0;  //Wroc na poczatek bufora jezeli blad
			}
			break;
			case 1:
			BUFOR[1] = i;  //Adres
			pozycja_BUFOR++;
			break;
			case 2:
			BUFOR[2] = i;  //Rozkaz
			dataFrame2=i;
			pozycja_BUFOR++;
			break;
			case 3:
			BUFOR[3] = i;  //Dane
			dataFrame3=i;
			pozycja_BUFOR++;
			break;
			default:
			pozycja_BUFOR = 0; //Jezeli blad odbioru
			break;
		
			//USART_puts(USART1, BUFOR);
		}
	
	}
	}
}

void InitializeLEDs()
{
	GPIO_InitTypeDef gpioStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    gpioStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    gpioStructure.GPIO_Mode = GPIO_Mode_AF;
    gpioStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &gpioStructure);
}


void delay(uint32_t ms) {
    ms *= 3360;
    while(ms--) {
        __NOP();
    }
}

int main()
{
	uint32_t i;
	InitializeLEDs();
	InitializeTimer();
	InitializePWMChannel();
	init_USART1(9600); // initialize USART1 @ 9600 baud

  USART_puts(USART1, "Init complete! Hello World!rn"); // just send a message to indicate that it works

	for (;;)
	{
		TIM4->CCR1 = dataFrame3+1000;
			TIM4->CCR2 = dataFrame3+1000;
			TIM4->CCR3 = dataFrame3+1000;
			TIM4->CCR4 = dataFrame3+1000;
		/*for(i=1008;i<1900;i++){
			delay(6);
		TIM4->CCR1 = i;
			TIM4->CCR2 = i;
			TIM4->CCR3 = i;
			TIM4->CCR4 = i;
	}
		for(i=1900;i>1008;i--){
			delay(6);
		TIM4->CCR1 = i;
			TIM4->CCR2 = i;
			TIM4->CCR3 = i;
			TIM4->CCR4 = i;
	}*/
}
}
//#include <stm32f4xx.h>
//#include <stm32f4xx_gpio.h>
//#include <stm32f4xx_rcc.h>
//#include <stm32f4xx_usart.h>
//#include <stm32f4xx_tim.h>

//void Delay(__IO uint32_t nCount) {
//  while(nCount--) {
//  }
//}

///* This funcion shows how to initialize
// * the GPIO pins on GPIOD and how to configure
// * them as inputs and outputs
// */
//void init_GPIO(void){

//	/* This TypeDef is a structure defined in the
//	 * ST's library and it contains all the properties
//	 * the corresponding peripheral has, such as output mode,
//	 * pullup / pulldown resistors etc.
//	 *
//	 * These structures are defined for every peripheral so
//	 * every peripheral has it's own TypeDef. The good news is
//	 * they always work the same so once you've got a hang
//	 * of it you can initialize any peripheral.
//	 *
//	 * The properties of the periperals can be found in the corresponding
//	 * header file e.g. stm32f4xx_gpio.h and the source file stm32f4xx_gpio.c
//	 */
//	GPIO_InitTypeDef GPIO_InitStruct;

//	/* This enables the peripheral clock to the GPIOD IO module
//	 * Every peripheral's clock has to be enabled
//	 *
//	 * The STM32F4 Discovery's User Manual and the STM32F407VGT6's
//	 * datasheet contain the information which peripheral clock has to be used.
//	 *
//	 * It is also mentioned at the beginning of the peripheral library's
//	 * source file, e.g. stm32f4xx_gpio.c
//	 */
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

//	/* In this block of instructions all the properties
//	 * of the peripheral, the GPIO port in this case,
//	 * are filled with actual information and then
//	 * given to the Init function which takes care of
//	 * the low level stuff (setting the correct bits in the
//	 * peripheral's control register)
//	 *
//	 *
//	 * The LEDs on the STM324F Discovery are connected to the
//	 * pins PD12 thru PD15
//	 */
//	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12; // we want to configure all LED GPIO pins
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; 		// we want the pins to be an output
//	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 	// this sets the GPIO modules clock speed
//	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 	// this sets the pin type to push / pull (as opposed to open drain)
//	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; 	// this sets the pullup / pulldown resistors to be inactive
//	GPIO_Init(GPIOD, &GPIO_InitStruct); 			// this finally passes all the values to the GPIO_Init function which takes care of setting the corresponding bits.

//	/* This enables the peripheral clock to
//	 * the GPIOA IO module
//	 */
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

//	/* Here the GPIOA module is initialized.
//	 * We want to use PA0 as an input because
//	 * the USER button on the board is connected
//	 * between this pin and VCC.
//	 */
//	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;		  // we want to configure PA0
//	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN; 	  // we want it to be an input
//	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;//this sets the GPIO modules clock speed
//	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;   // this sets the pin type to push / pull (as opposed to open drain)
//	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;   // this enables the pulldown resistor --> we want to detect a high level
//	GPIO_Init(GPIOA, &GPIO_InitStruct);			  // this passes the configuration to the Init function which takes care of the low level stuff
//}

//int main(void){
//// this counter is used to count the number of button presses
//  uint8_t i = 0;
//  // initialize the GPIO pins we need
//  init_GPIO();

//  /* This flashed the LEDs on the board once
//   * Two registers are used to set the pins (pin level is VCC)
//   * or to reset the pins (pin level is GND)
//   *
//   * BSRR stands for bit set/reset register
//   * it is seperated into a high and a low word (each of 16 bit size)
//   *
//   * A logical 1 in BSRRL will set the pin and a logical 1 in BSRRH will
//   * reset the pin. A logical 0 in either register has no effect
//   */
//  GPIOD->BSRRL = 0xF000; // set PD12 thru PD15
//  Delay(1000000L);		 // wait a short period of time
//  GPIOD->BSRRH = 0xF000; // reset PD12 thru PD15

//  

//  while (1){

//		/* Every GPIO port has an input and
//		 * output data register, ODR and IDR
//		 * respectively, which hold the status of the pin
//		 *
//		 * Here the IDR of GPIOA is checked whether bit 0 is
//		 * set or not. If it's set the button is pressed
//		 */
//		if(GPIOA->IDR & 0x0001){
//			// if the number of button presses is greater than 4, reset the counter (we start counting from 0!)
//			if(i > 3){
//				i = 0;
//			}
//			else{ // if it's smaller than 4, switch the LEDs

//				switch(i){

//					case 0:
//						GPIOD->BSRRL = 0x1000; // this sets LED1 (green)
//						GPIOD->BSRRH = 0x8000; // this resets LED4 (blue)
//						break;

//					case 1:
//						GPIOD->BSRRL = 0x2000; // this sets LED2 (orange)
//						GPIOD->BSRRH = 0x1000; // this resets LED1
//						break;

//					case 2:
//						GPIOD->BSRRL = 0x4000; // this sets LED3 (red)
//						GPIOD->BSRRH = 0x2000; // this resets LED2
//						break;

//					case 3:
//						GPIOD->BSRRL = 0x8000; // this sets LED4
//						GPIOD->BSRRH = 0x4000; // this resets LED3
//						break;
//					}

//				i++; // increase the counter every time the switch is pressed
//			}
//			Delay(3000000L); // add a small delay to debounce the switch
//		}
//	}
//}

