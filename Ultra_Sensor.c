#include "Ultra_Sensor.h"
#include "stm32f4xx.h" 

/*================================
#define trig 12 //GPIOD,output   =
#define echo 1 //GPIOA,input		 =
==================================
*/

void sensor_init(void)
{
		//enables GPIOD in AHB1ENR
	RCC->AHB1ENR |= (1u<<3);
	//enables GPIOA in AHB1ENR
	RCC->AHB1ENR |= (1u<<3);
	
	//clears to make sure the pins are 00-input
	GPIOD -> MODER &= ~(3u<<2*trig);
	GPIOA -> MODER &= ~(3u<<2*echo);
	//sets it to output
	GPIOD -> MODER |= (1u<<2*trig);

	//makes the pins push-pull, which is the reset state
	GPIOA -> OTYPER &= ~(3u<<2*echo);
	
	
	
}





