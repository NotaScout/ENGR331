#include "stm32f4xx.h" 

#include "LED.h"



void LED_init(void){ 
 LED_ORANGE_enable();
 LED_BLUE_enable();
 LED_GREEN_enable();
 LED_RED_enable();
} 


void LED_ON(int LED){
	
	GPIOD -> ODR |= (1u<<LED);
	
}

void LED_OFF(int LED){
	
	GPIOD -> ODR &= ~(1u<<LED);
	
}

void LED_toggle(int LED){
	GPIOD->BSRR ^= (1<<(LED+16));

	
	
	
}


void LED_ORANGE_enable(){

	RCC->AHB1ENR |= (1<<3);/* per -> reg*/
		GPIOD -> MODER &=  ~(3u<<2*LED_ORANGE);
	GPIOD -> MODER |= (1<<2*LED_ORANGE); /* OR mask*/
	GPIOD -> OTYPER &= ~(1u<<LED_ORANGE);/* AND mask*/
	GPIOD -> ODR |= (0<<LED_ORANGE);
}

void LED_RED_enable(){
		RCC->AHB1ENR |= (1<<3);/* per -> reg*/
		GPIOD -> MODER &=  ~(3u<<2*LED_RED);
	GPIOD -> MODER |= (1<<2*LED_RED); /* OR mask*/
	GPIOD -> OTYPER &= ~(1u<<LED_RED);/* AND mask*/
	GPIOD -> ODR |= (0<<LED_RED);
}

void LED_BLUE_enable(){
		RCC->AHB1ENR |= (1<<3);/* per -> reg*/
		GPIOD -> MODER &=  ~(3u<<2*LED_BLUE);
	GPIOD -> MODER |= (1<<2*LED_BLUE); /* OR mask*/
	GPIOD -> OTYPER &= ~(1u<<LED_BLUE);/* AND mask*/
	GPIOD -> ODR |= (0<<LED_BLUE);
}

void LED_GREEN_enable(){
	
	
		RCC->AHB1ENR |= (1<<3);/* per -> reg*/
		GPIOD -> MODER &=  ~(3u<<2*LED_GREEN);
	GPIOD -> MODER |= (1<<2*LED_GREEN); /* OR mask*/
	GPIOD -> OTYPER &= ~(1u<<LED_GREEN);/* AND mask*/
	GPIOD -> ODR |= (0<<LED_GREEN);
}