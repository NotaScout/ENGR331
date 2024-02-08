#include "stm32f4xx.h" 

#include "RPG.h"
#include "LCD.h"
#include "LED.h"
#include "Motor.h"

void Motor_port_init(void){
// init //
	RCC->AHB1ENR |= (1u<<3);
	
	//clears to make sure the pins are 00-input
	GPIOD -> MODER &= ~(3u<<2*Hbridge1A_4A);
	GPIOD -> MODER &= ~(3u<<2*Hbridge2A_3A);

	// set to output 01
	GPIOD -> MODER |= (1u<<2*Hbridge1A_4A);
	GPIOD -> MODER |= (1u<<2*Hbridge2A_3A);

	//makes the pins push-pull
	GPIOD -> OTYPER &= ~(3u<<2*Hbridge1A_4A);
	GPIOD -> OTYPER &= ~(3u<<2*Hbridge2A_3A);

	//
	
	
	// unsure if needed
	//GPIOD -> PUPDR |= (1u<<2*BUT);
	

}

// NOTE: this assumes that these ports are next to each other
// 1234 A
void Motor_forward(void){
GPIOD -> ODR &= ~(3u<<Hbridge2A_3A); // 00
GPIOD -> ODR |= (1u<<Hbridge1A_4A); // 10
} // set 1001
void Motor_backward(void){
GPIOD -> ODR &= ~(3u<<Hbridge2A_3A); // 00
GPIOD -> ODR |= (1u<<Hbridge2A_3A); // 01
} // set 0110
void Motor_stop(void){
GPIOD -> ODR &= ~(3u<<Hbridge2A_3A); // 00

} // 0000




