#include "initialize.h"
extern char dataFrame2;
extern char dataFrame3;
extern int whoAmI;
struct servo servo;
int main(){
	volatile int connectionCheck=1;
	volatile uint8_t connectionArray[20]={0xFF,0x01,0xFF,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x0A};
	volatile unsigned int position;
	initializeGPIOs();
	initializeTimers();
	initializePWMChannels();
	initializeUSART(9600); // initialize USART1 @ 9600 baud
USART_puts(USART1, "OK");
	//USART_puts(USART1, "Init complete!"); // just send a message to indicate that it works

	for (;;)	{
		if(whoAmI>0&&connectionCheck){
for(position=0;position<=19;position++){
	while(!(USART1->SR & 0x00000040));
			USART_SendData(USART1,connectionArray[position]);
}
		connectionCheck=0;
			}
		  TIM4->CCR1 = 10*servo.servo_1+1000;
			TIM4->CCR2 = 10*servo.servo_2+1000;
			TIM4->CCR3 = 10*servo.servo_3+1000;
			TIM4->CCR4 = 10*servo.servo_4+1000;
		TIM2->CCR1 = 10*servo.servo_5+1000;
			TIM2->CCR2 = 10*servo.servo_6+1000;
			TIM2->CCR3 = 10*servo.servo_7+1000;
			TIM2->CCR4 = 10*servo.servo_8+1000;
		TIM3->CCR1 = 10*servo.servo_9+1000;
			TIM3->CCR2 = 10*servo.servo_10+1000;
			TIM3->CCR3 = 10*servo.servo_11+1000;
			TIM3->CCR4 = 10*servo.servo_12+1000;
		TIM5->CCR1 = 10*servo.servo_13+1000;
			TIM5->CCR2 = 10*servo.servo_14+1000;
			TIM5->CCR3 = 10*servo.servo_15+1000;
			TIM5->CCR4 = 10*servo.servo_16+1000;
			//delay(10);
}
}

