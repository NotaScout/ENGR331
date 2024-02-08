#include "stm32f4xx.h"
#include "ADC.h"
#include "Timer.h"
#include "LED.h"

int myData[500];
void ADC_init(void){

RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN);
RCC->APB2ENR |= (RCC_APB2ENR_ADC1EN);// enables adc
	GPIOA->MODER |=(1u<<(2*adc+1));
	GPIOA->MODER |=(1u<<2*adc); // sets to 11 (analog), pin 1

ADC1->CR1 &= ~(3u<<24); //12-bit resolution which is default
	
	//smaple time of 112
ADC1->SMPR2  |=(1u<<6);
	ADC1->SMPR2  |=(1u<<8);
	ADC1->SMPR2 &= ~(1u<<7);
	//make sure it is not cont
	ADC1->CR2 &= ~(1u<<1);
	
	// Specify channel number 1 of the 1st conversion
	ADC1->SQR3 |= (adc)<<0;
	
	
		
		
}

void ADC_Poll(void){

	//ADC conversion ON bit 
	ADC1->CR2 |= (1u<<0);
	
	//start ADC conversion SWSTART
	ADC1->CR2 |= (1u<<30);
	
	//waits unit end of conversion (EOC) flag is set
	while((ADC1->SR & (1u<<1)) == 0){}
	
	//reads data reg
		
		

			//LED_toggle(LED_GREEN);
			myData[i]=ADC1->DR ;
			//ADC1->SR &= ~(1u<<1);
			

	
	
}
