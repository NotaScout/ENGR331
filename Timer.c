#include "Timer.h"
#include "stm32f4xx.h" 
#include "LED.h"
#include "LCD.h"
#include "RPG.h"
#include "ADC.h"
#include "accelerometer.h"

int START=0;
int END=0;
float TIME;// the extern is created in this file, so that other files may se it.
float DIST=0;
int i=0;

void TIM6_init(uint32_t  PSC_num, uint32_t  ARR_num)
{
	/* enables clock*/
	RCC -> APB1ENR |= (RCC_APB1ENR_TIM6EN);
	TIM6 ->CNT =0;
	TIM6 ->PSC =PSC_num;
	TIM6 ->ARR =ARR_num;
	TIM6 ->DIER |= 1<<0; //enable UIE for interrupt
	// starts the counter
	TIM6 ->CR1 |= 1<<0;//bit 0 of CR1 is count enable
	NVIC_SetPriority(TIM6_DAC_IRQn,0); //sets priority, if all are set to 0, than all share the same priority
	NVIC_ClearPendingIRQ(TIM6_DAC_IRQn); //clears pending
	NVIC_EnableIRQ(TIM6_DAC_IRQn); //enables
	
	__enable_irq();
	
}

void TIM4_init(uint32_t DUTY_num, uint32_t PERIOD)
{
	
	/* enables clock*/
	RCC -> APB1ENR |= (RCC_APB1ENR_TIM4EN);
	
	//enables GPIOD in AHB1ENR
	RCC->AHB1ENR |= (1u<<3);
	
	//set alternate functonof PD15 for motor
	GPIOD->MODER |= (1u<<31); //sets the pin to alt mode, this line sets bit 31
	GPIOD->MODER &= ~(1u<<30);	//this line clears pit 30 for to make sure it is 10 for alt
	GPIOD->AFR[1] |= (1u<<29);	//AFR[1] cause port 15 is on the high reg, assumes that bits around 29 are zero, AFRH is the high reg
	
	//set alternate functonof PD12 for ultra_sensor
	GPIOD->MODER |= (1u<<25); 
	GPIOD->MODER &= ~(1u<<24);	
	GPIOD->AFR[1] |= (1u<<17);//puts it on af2	//*/
	
	
	// set CC4 channel to output mode (default after reset)
	//this is defult mode
	TIM4->CCMR2 &= ~(1u<<9 | 1u<<8);//pin 15 - channel 4 - output
	
	TIM4->CCMR1 &= ~(1u<<1 | 1u<<0);//sets CC1 to be an output, pin 12- channel 1
	
	//b) Select the polarity by writing the CCxP bit in CCER register.
	TIM4->CCER &=~(1u<<13); // CH4
	
	//which is the same as TIM4->CCER &=~TIM_CCER_CC4P;
	
	
	//sets PWM mode (PWM1 or PWM2) by writing OCxM bits into CCRMx reg
	TIM4->CCMR2 |= (1u<<14) |(1u<<13);//pin15
	TIM4->CCMR2 &=~ (1u<<12);//sets PWM1 Mode
	
	TIM4->CCMR1 |= (1u<<6) | (1u<<5);//pin12
	TIM4->CCMR1 &=~ (1u<<4);//sets PWM1 Mode
	
	//Drive the transmit pin of the sensor with PWM signal with a period between 0.65sec to 1sec and a high pulse of 10us.
	//16MHz clk/PSC = runs the counter at 1kHz=1ms THIS IS THE "RESOLUTION" 1 = 1 STEP SKIPED, so resolution is divided in half
	//TIM4 ->PSC =16000000/100000 -1;  //159, gets you 10 us
	
	//TIM4 ->PSC =159;
	TIM4 ->PSC =15;
	
	TIM4 ->ARR = PERIOD; //affects both channels, when it was 100, motor worked perfectly
	TIM4->CCR4 = DUTY;//DUTY, it is on channel 4 m
	
	TIM4->CCR1 = 10;//channel 1
	
	//preload bit into CCMRx reg and the ARPE bit into CR1 reg
	TIM4->CCMR2 |= (1u<<11); //CR4preload
	TIM4->CR1 |= (1u<<7);//ARR preload, handles all pins
	//pin12
	TIM4->CCMR1 |= (1u<<3); //CR4preload, oc1pe bit
	
	
	//enable compare output 4
	
	TIM4->CCER |= 1u<<12; //CC4E bit. pin15
	TIM4->CCER |= 1u<<0; //CC1E bit, pin12
	
	// enable count
	TIM4->CR1 |= 1u<<0;//enable count
}

//trig is the putput and is on GPIOD, treat it like you are just turning on the green led.
void TIM2_init(uint32_t  DUTY_num, uint32_t  ARR_num)
{
	DUTY_num=1;//to get rid of warnings
	ARR_num =DUTY_num;
	DUTY_num=ARR_num;//*/
	/* enables clock*/
	RCC -> APB1ENR |= (RCC_APB1ENR_TIM2EN);
	RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	
	//sets up alternate mode.
	GPIOA->MODER &= ~(3u<<1*2);
	GPIOA->MODER |= (2u<<1*2);
	
	GPIOA->AFR[0] |=( 1<<1*4);//routes MUX
	
	// set CC4 channel to output mode (default after reset) 
	
	TIM2->CCMR2 &= ~(1u<<9 | 1u<<8);
	//b) Select the polarity by writing the CCxP bit in CCER register.
	TIM2->CCER &=~(1u<<13); 
	//which is the same as TIM4->CCER &=~TIM_CCER_CC4P;
	
	
	//sets PWM mode (PWM1 or PWM2) by writing OCxM bits into CCRMx reg
	TIM2->CCMR2 |= (1u<<14) |(1u<<13);
	TIM2->CCMR2 &=~ (1u<<12);//sets PWM1 Mode
	
	
	
	//TIM2 ->PSC =16000000/1000 -1; //16MHz clk/PSC = runs the counter at 1kHz=1ms
	//TIM2 ->ARR = ARR_num; //period
	//TIM2->CCR2 = DUTY_num;//DUTY
	
	//set rising edge and falling edge for channel 2
	TIM2->CCER |= (1<<5 | 1<<7);
	
	TIM2->CCMR1 |=(1<<8);
	TIM2->CCER |=(1<<4);//enables channel
	
	
	//preload bit into CCMRx reg and the ARPE bit into CR1 reg
	TIM2->CCMR2 |= (1u<<11); //CR4preload
	TIM2->CCMR1 |= (1u<<7);//ARR preload
	
	
	/*
	//enable compare output 4
	
	TIM2->CCER |= 1u<<12; //CC4E bit
	*/

	
	
	//enable interrupts
	
	TIM2->DIER |= (1<<2);// for edges
	TIM2->DIER |= (1<<0);//enaables overflow
	
	
	
	// enable count
	TIM2->CR1 |= 1u<<0;//enable count
	
	
	__enable_irq();	
	NVIC_SetPriority(TIM2_IRQn,0);
	NVIC_ClearPendingIRQ(TIM2_IRQn);
	NVIC_EnableIRQ(TIM2_IRQn); // Enable interrupt from TIM2 (NVIC level)
}

void TIM3_init(uint32_t DUTY_num, uint32_t  ARR_num){
	/* enables clock*/
	RCC -> APB1ENR |= (RCC_APB1ENR_TIM3EN);
	RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	
	
	//set input functon of PA6 for ADC
	//GPIOA->MODER &= ~(3u<<pin6*2);	
	//GPIOA->AFR[0] |= (1u<<25);	//AFR[0] cause port 6 is on the low reg, check ref man 272 for help

	// set CC1 channel to output mode (default after reset)
	//this is defult mode
	//TIM3->CCMR1 &= ~(1u<<1 | 1u<<0);//pin 6 - channel 1 - output

	//tim time controls
	TIM3 ->PSC =40;
	TIM3 ->ARR = ARR_num; //affects both channels, when it was 100, motor worked perfectly
	//TIM3->CCR1 = DUTY_num;//DUTY, it is on channel 4 m
	TIM3->CCR1 = 1;//channel 1
	TIM3->CCER &=~(1u<<1); // CH1
	TIM3->CCER |=(1u<<0); // CH1

//preload bit into CCMRx reg and the ARPE bit into CR1 reg
	TIM3->CCMR2 |= (1u<<11); //CR4preload
	TIM3->CR1 |= (1u<<7);//ARR preload, handles all pins

	

TIM3->DIER |=(1u<<0);// needed when using interrupt service

	// enable count
	TIM3->CR1 |= 1u<<0;//enable count
	//needs the UIE flag high in DIER
	NVIC_SetPriority(TIM3_IRQn,0); //sets priority, if all are set to 0, than all share the same priority
	NVIC_ClearPendingIRQ(TIM3_IRQn); //clears pending
	NVIC_EnableIRQ(TIM3_IRQn); //enables
	__enable_irq();
}






//=====================================interrupt routines============================
/* YOU DONT CALL THIS< THE CHIP DOES
 void  TIM6_DAC_IRQHandler(void){ //his is also LED_RED underlined, SR = status reg
	 NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
	 if(TIM6->SR & TIM_SR_UIF){
	 GPIOD ->ODR ^= 1<<LED_LED_BLUE;//LED_toggles LED_BLUE LED
	 TIM6 ->SR &=~(1u<<0); //clears UIF flag in SR
	 } 
 }
*/

void TIM6_interrupt_int(void)//enables
{
	__enable_irq();	
	NVIC_SetPriority(TIM6_DAC_IRQn,0);
	NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
	NVIC_EnableIRQ(TIM6_DAC_IRQn); // Enable interrupt from TIM6 (NVIC level)
	
	
	
	
}

//=============================================================================
// TIM6 Interrupt Handler
//=============================================================================
void TIM6_DAC_IRQHandler(void)
{
	NVIC_ClearPendingIRQ(TIM6_DAC_IRQn);
	if(TIM6->SR & TIM_SR_UIF){ // if UIF flag is set -- Just in case
		TIM6->SR &= ~TIM_SR_UIF; // clear UIF flag
	}
	LED_toggle(LED_BLUE);
		ACC_Read(); //reads the x,y,z values from acc

}

void TIM2_IRQHandler(void)
{
	static int OVFLOW=0; // only gets int once, othwerwise it will stick around.
	NVIC_ClearPendingIRQ(TIM2_IRQn);
	if(TIM2->SR & TIM_SR_UIF){ // if UIF flag is set -- Just in case
		TIM2->SR &= ~TIM_SR_UIF; // clear UIF flag
		OVFLOW=OVFLOW+1;
	}
	if(TIM2-> SR & TIM_SR_CC2IF){// if CH 2 flag is set, which means if a rising/falling edge is dected
		//may need a very short delay here if it does not work
		//for(int j=0; j<=100000; j++){}
		if(GPIOA->IDR  & (1u<<receive))//if the signal is high than after edge dection 
		{
			START= TIM2->CCR2; // grabs the value of time from TIM2 
		}
		else //if the if statement is false, than
		{
			END=TIM2->CCR2; // grabs the value of time from TIM2 , BUT at a differnt time
		}
		
		TIM2->SR &= ~TIM_SR_CC2IF;//clears flag
	}
	//calculates the distance 
	TIME= (END + OVFLOW*0xFFFF)-START;
	OVFLOW=0;
	//distance in inch replace 68 with 148
	//distance in cm DIST= PulseWidth(us)/58
	DIST= (TIME)/580;
	LCD_clear();
	LCD_send_String("DISTANCE=");
	LCD_place_cursor(0xC0); // New line
	LCD_send_floatString(DIST); // display distance Float
	// need to multiply set set duty cycle by distance multiplier
	
	float DUTY_OUT;
// duty cycle is distance divided by a number( we can decide).
	DIST= (TIME)/58;
	DUTY_OUT=(DIST/50)*DUTY;// converts DIST to DUTY which gets a range of 5 to about 200
	//upper bound DUTY at 100
	if(DUTY_OUT > 65000)
	{
		DUTY_OUT = 65000;
	}
	//lower bound duty at 0 // set maybe to 1000 or similar
	if(DIST < 4) //
	{
		DUTY_OUT = 0; // dont change
	}
	TIM4->CCR4 = DUTY_OUT;
	//DUTY= 30; //this sets duty a set number for the entire run time
	
}



void TIM3_IRQHandler(void){
NVIC_ClearPendingIRQ(TIM3_IRQn);
	if(TIM3->SR & TIM_SR_UIF){ // if UIF flag is set -- Just in case
		
		
		if(i<500){
		ADC_Poll();
		}
		else
		{
			i=1000;
		}
		
		i=i+1;
		TIM3->SR &= ~TIM_SR_UIF; // clear UIF flag
	}
}





