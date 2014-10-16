#include "initialize.h"
extern int dataFrame2;
extern int dataFrame3;
extern int whoAmI;
int main(){
	volatile int connectionCheck=1;
	volatile uint8_t connectionArray;
	initializeGPIOs();
	initializeTimers();
	initializePWMChannels();
	initializeUSART(9600); // initialize USART1 @ 9600 baud

	//USART_puts(USART1, "Init complete!"); // just send a message to indicate that it works

	for (;;)	{
		if(whoAmI>0){
			if(connectionCheck==1){
				connectionArray=0xFF;
			USART_SendData(USART1,connectionArray);
				connectionCheck=-1;
			}
		TIM4->CCR1 = dataFrame3+1000;
			TIM4->CCR2 = dataFrame3+1000;
			TIM4->CCR3 = dataFrame3+1000;
			TIM4->CCR4 = dataFrame3+1000;
		TIM2->CCR1 = dataFrame3+1000;
			TIM2->CCR2 = dataFrame3+1000;
			TIM2->CCR3 = dataFrame3+1000;
			TIM2->CCR4 = dataFrame3+1000;
		TIM3->CCR1 = dataFrame3+1000;
			TIM3->CCR2 = dataFrame3+1000;
			TIM3->CCR3 = dataFrame3+1000;
			TIM3->CCR4 = dataFrame3+1000;
		TIM5->CCR1 = dataFrame3+1000;
			TIM5->CCR2 = dataFrame3+1000;
			TIM5->CCR3 = dataFrame3+1000;
			TIM5->CCR4 = dataFrame3+1000;
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
}
