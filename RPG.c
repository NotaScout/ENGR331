#include "stm32f4xx.h" 

#include "RPG.h"
#include "LCD.h"
#include "LED.h"
#include "Motor.h"

int DUTY;

int DUTY_Increment(void){
int x = 6500; // set duty cycle increment // needs to be a multiple of the  period
	return x;
}

// A=pin 8 and B=pin 9
void RPG_port_init(){
	//enables GPIOD in AHB1ENR
	RCC->AHB1ENR |= (1u<<3);
	
	//clears to make sure the pins are 00-input
	GPIOD -> MODER &= ~(3u<<2*A);
	GPIOD -> MODER &= ~(3u<<2*B);
	GPIOD -> MODER &= ~(3u<<2*BUT);
	//makes the pins push-pull
	GPIOD -> OTYPER &= ~(3u<<2*A);
	GPIOD -> OTYPER &= ~(3u<<2*B);
	GPIOD -> OTYPER &= ~(3u<<2*BUT);
	
	GPIOD -> PUPDR |= (1u<<2*BUT);

	
	
}

void Signal_A_interrupt_init(){
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; 
	
// setting up the EXTI/GPIO lines to YELL out interrupt 
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // 1<<14 
	
	//change
	SYSCFG->EXTICR[2] &=~(0xFu);//clear|| EXTICER 1 wll be EXTICR[0], so for e it will be [2]
	SYSCFG->EXTICR[2] |= (0x3u);//sets it exti8 to 0011 (PD)
	EXTI->IMR |= 1u<<A;  // Interrupt Mask reg
	EXTI->RTSR |= 1u<<A; // Rising trigger selection reg
	
	//might need change
	NVIC_SetPriority(EXTI9_5_IRQn,0); 
	NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
	NVIC_EnableIRQ(EXTI9_5_IRQn); 
	
	//enables main processor interrupt
	__enable_irq();  
	
}

//interrupt leads here
void EXTI9_5_IRQHandler(){  
// make this short and sweet 
	//uint32_t DUTY;
	//uint32_t reverse;
	int forward; // possible just use 1 - bit reverse instead of 2 bits for forward & reverse
	// FUTURE NOTE; I have done just that, Forward is now an internal signal
	// inits // 
		LCD_clear();
	char inputstrDIR1[] = "DIR=";
		LCD_send_String(inputstrDIR1);
	LCD_place_cursor(0xC0);
	// set to display
	
	
	// actual code for setting the direction & duty cycle
	
	

	
if (DUTY == 0){ //sets direction if duty_cycle == 0
if((GPIOD -> IDR & 1u<<B) == 0){
forward =1;
DUTY = DUTY + DUTY_Increment();
char inputstrDIR[] = "L";
	LCD_send_String(inputstrDIR);
	Motor_forward();
}
else if((GPIOD -> IDR & 1u<<B) != 0){
forward = 0;
DUTY = DUTY + DUTY_Increment();
char inputstrDIR[] = "R";
	LCD_send_String(inputstrDIR);
	Motor_backward();
}

}
else if(forward){ // checks what to do  if forward
if((GPIOD -> IDR & 1u<<B) == 0){ // Left
DUTY = DUTY + DUTY_Increment();
char inputstrDIR[] = "L";
	LCD_send_String(inputstrDIR);
}
else if((GPIOD -> IDR & 1u<<B) != 0){ //Right
DUTY = DUTY - DUTY_Increment();
char inputstrDIR[] = "R";
	LCD_send_String(inputstrDIR);
}

}
else if(~forward) // checks what to do when backward
{
if((GPIOD -> IDR & 1u<<B) == 0){ // Left
DUTY = DUTY - DUTY_Increment();
char inputstrDIR[] = "L";
	LCD_send_String(inputstrDIR);
}
else if((GPIOD -> IDR & 1u<<B) != 0){ //Right
DUTY = DUTY + DUTY_Increment();
char inputstrDIR[] = "R";
	LCD_send_String(inputstrDIR);
}
}


// write in the rest here

	
	// if((GPIOD -> IDR & 1<<B) != 0) || this checks the dirction that has been turned (L | R)
	// (GPIOD -> IDR & LED_RED) == 0) || this checks the LED_RED LED || maybe used

	
	/*
if((GPIOD -> IDR & 1<<B) != 0){//if true than right
	if((GPIOD -> IDR & LED_RED) == 0){//forward{
		char inputstrDIR[] = "R";
		LCD_send_String(inputstrDIR);
		DUTY=DUTY+10;
	  }
	}
	else if((GPIOD -> IDR & B)==0)//left check
		{// if true than left
	 if((GPIOD -> IDR & LED_RED) == 0){//forward
		char inputstrDIR2[] = "L";
		LCD_send_String(inputstrDIR2);
		 DUTY=DUTY-10;
	   }
}    


*/


		//======= caps DUTY to the bounds of 0 - 100=========
	if(DUTY > 10*DUTY_Increment() && DUTY <15*DUTY_Increment()){//caps at 100
			DUTY = 10*DUTY_Increment();
	}

	TIM4->CCR4 = DUTY;// sends the DUTY out of the interrupt
	LCD_send_intString(DUTY);
	
	NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
	EXTI->PR |= (1u<<A);
} 

void RPG_Button_init(){
	//enables GPIOD in AHB1ENR
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN; 
	
// setting up the EXTI/GPIO lines to YELL out interrupt 
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // 1<<14 
	
	//change
	SYSCFG->EXTICR[2] &=~(0xFu<<8);//clear|| EXTICER 1 wll be EXTICR[0], so for e it will be [2]
	SYSCFG->EXTICR[2] |= (0x3u<<8);//sets it exti8 to 0011 (PD)
	EXTI->IMR |= 1u<<BUT;  // Interrupt Mask reg
	EXTI->RTSR &= ~(1u<<BUT); // Rising trigger selection reg
	EXTI -> FTSR  |= 1u<<BUT; //sets to falling trigger
	
	//might need change
	NVIC_SetPriority(EXTI15_10_IRQn,0); 
	NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
	NVIC_EnableIRQ(EXTI15_10_IRQn); 
	
	//enables main processor interrupt
	__enable_irq();  
	
	
}

/*
What this should do:
Stop the motor by setting enable pin on H-bridge to 0 and or sending 1100,0011, or 0000 signal to transistor inputs

Ideally send 0000 and EN to 0

Optional:
write STOPPED in some form on the display out


*/
void  EXTI15_10_IRQHandler(void){ // RPG button interrupt
	//uint32_t DUTY = 0;
	LCD_clear();
	LCD_send_String("STOPPED");
	Motor_stop();
	TIM4->CCR4 = DUTY;
	NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
	EXTI->PR |= (1u<<BUT);
}

/*
if (DUTY == 0){ //sets direction if duty == 0
if(L){
forward =1;
DUTY = DUTY + 10;
}
else if(R){
forward = 0;
DUTY = DUTY + 10;
}

}
else if(forward){ // checks what to do  if forward
if(L){
DUTY = DUTY + 10;
}
else if(R){
DUTY = DUTY - 10;
}

}
else if(~forward) // checks what to do i backward
{
if(L){
DUTY = DUTY - 10;
}
else if(R){
DUTY = DUTY + 10;
}



}*/
