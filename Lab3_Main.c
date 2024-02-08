#include "stm32f4xx.h" 
#include <string.h>
#include "LCD.h"
#include "RPG.h"
#include <stdio.h>
#include "LED.h"
#include "Timer.h"
#include "Motor.h"
#include "Ultra_Sensor.h"
#include "ADC.h"
#include "accelerometer.h"

//TODO:
/*
increase how much the Duty goes up to get the motor to work

make function that automatically takes in strings and outputs to the display; intermediate step
also ones for numbers and s

stop START and END from always rising.

decide if we need a power regulator link: https://www.pololu.com/product/3541

============================================== acceleromter================== 

enable axis 7.20 CTRL_REG4 (20h) enabled by defult
ctrl_rg 0100 0111 //0100: 25 hz bit || 0-2:enable axis || bit 3  for contious update(defult
ctrl_reg6 bit6=1 enables FIFO


FIFO_SRC has watermark control CTRL_REG6(25) enables it		||			GPIOD -> ODR &= ~(0xFu) << DB4;
OUTS1 has innterupts out regs

read STATUS(27) for bit 3, goes high if set of data is high						 data2[#] &=	~(0xF7) << (0000 1000) 

// do NOT write new lcd control functions in here//

Done;
write string
LCD_clear(); fcn
make lcd cursor placement take a single int
write reset fcn (should put the cursor on 1,1 and clear display)
"LCD_send_intString"

*/
//===============Variable notes=======================
// int(no 0.#)/float(6 # after decimal)/double or ifd
//
//static ifd = 0; makes the variable static. It would be created twice.
//	if a function gets called multiple times, the variable will only be set to 0 once.
//
//extern ifd; look at the variable: TIME. 
//	Mentioned it is an extern in Timer.h, declared in Timer.c, and can be used in all files that have Timer.h included
//
//if you add a global variable in a header file, all files that use said header flile
//	will create their own and distinct variables that happen to use the same name
//====================================================

// =============Global Variables=======================
	//uint32_t DUTY =0;//should be the starting value of duty cycle // this value 0 should start with 0% on 
	
//=====================================================

// Standardize naming conventions!
// Naming should be as such : 
// subject_action();
int main(void){
	//=============Variables============================
	//TIME = 0; // is how you would use an extern variable.
	DUTY = 0;//this is now a shared variable across all files that have RPG.h included
	i=0;
	data2[0]=0;
	
	//=============starts everyting=====================
	ADC_init();
	
	LED_init();
	LED_ON(LED_BLUE);
	SPI_Port_Init();
	SPI_Init();//after port init
	ACC_Init();
	TIM6_init(99999, 15999); //psc, arr for a 10 ms 
	//TIM2_init(0,0); //as of lab 6, these #'s will not be used.
	TIM3_init(2,50); //DUTY_num, ARR_num
	//TIM4_init(DUTY, 65000); //(DUTY, PERIOD/ARR),65000 with PSC=159 gets you a high of 10 us and period of 651 ms, DUTY DOES NOT AFFECT 10 us
	
	//===================RESET STATE====================

	//===============code for lab=======================
		
	

	
		
		setCS(1); // initialize CS to 1
		
	// set CS to low
		setCS(0);//to send data
		///	register address=\/    \/=command
		data2[0]= SPI_Send(0x0F | READ ); // who am i command!
		data2[1] = SPI_Send(0x0); // dummy byte - to receive 16 bit from peripheral
		setCS(1);

	
	
	
	
	
	
	
	
	while(1){}  
		
}

//what files were changed( turn in files that were changed)
// timer, ultra_sensor

/*
	RPG_port_init();
	Signal_A_interrupt_init();
	RPG_Button_init();
	LED_init();
	LCD_port_init();
LCD_clear();
	Motor_port_init();
	LCD_send_String("STOPPED");
	Motor_stop();
	LCD_init();

*/

