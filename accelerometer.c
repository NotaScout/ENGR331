#include "accelerometer.h"
#include "stm32f4xx.h" 
 uint8_t data2[10];
 
/* Class Ex: SPI interface with onboard accelerometer
See page 6 of Discovery Board Schematic Package
MEMS Accelerometer : LIS3DSH
PIN Connections:
CS : PE3
SPI1_SCK : PA5
SPI1_MISO : PA6 gets data
SPI1_MOSI : PA7 send command
See page 19 (Section 6.7 Motion Sensor) in the Discovery Board Manual
Motion sensor
The ST-MEMS motion sensor is an ultra-compact low-power three-axis linear
accelerometer.
The motion sensor includes a sensing element and an IC interface able to provide
the measured acceleration to the external world through the I2C/SPI serial
interfaces.
The STM32F407VG microcontroller controls this motion sensor through the SPI
interface.
*/
#include "stm32f4xx.h" // Device header
void SPI_Port_Init(void);
void SPI_Init(void);
void SPI_Port_Init(){
	// 1. Enable GPIOA, GPIOE (in AHB1) and SPI1 (in APB2)
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOEEN;
		RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	// 2. CS is connected to PE3
	// set Port E Pin 3 as output (Moder = 01)
		GPIOE->MODER |= 1u<<6; //(bit 7=0; bit 6=1 for pin3 )
	// 3. set GPIOA Pins 5, 6 and 7 as alternate functions
		// SPI1 is set as Alternate Function #5 (0101)
		GPIOA->AFR[0] |= 0x55500000;
	// 4 set GPIOA pins 5,6,7 MODE register to alternate function (MODER = 10)
		GPIOA->MODER |= (1u<<11)|(1u<<13)|(1u<<15);
}
void SPI_Init(){
/*Generate Serial Clock (SCK) at the right frequency (set BR[2:0]).
Select CPOL and CPHA in SPI_CR1 register bits based on the specifications of the
peripheral
Set MSTR to 1 to set device as Controller
Select Data frame format (DFF)_: 8-bit (default) or 16 bit.
Set or clear SSM and SSI bits to configure the Chip Select pin in hardware or
software
Set SPE to enable SPI
*/
	//set lowerst sck freq
	RCC->APB2ENR |= (RCC_APB2ENR_SPI1EN);
		SPI1->CR1 |= (1u<<5) | (1u<<4) | (1u<<3);
		SPI1->CR1 |= (1u<<0) | (1u<<1); //CPOL = 1; CPHA=1;
	// Set MSTR to 1 to set device as Controller
		SPI1->CR1 |= (1u<<2);
		SPI1->CR1 |= (1u<<9); //ssm = 1
		SPI1->CR1 |= (1u<<8);
	//Set SPE to enable SPI
		SPI1->CR1 |= (1u<<6);
}


		uint8_t SPI_Send(uint8_t myData){
		uint8_t data_in_DR;
	// send SPI data on DR
		SPI1->DR = myData; //myData sent
	// wait for send to finish
			
		while(!(SPI1->SR & SPI_SR_RXNE));
	//		while(!(SPI1->SR & SPI_SR_RXNE));
	// while(!(SPI1->SR & SPI_SR_TXE));
		data_in_DR= SPI1->DR; // data received
		return data_in_DR;
}
		
void setCS(uint8_t HIGH_LOW){
	// function sets CS signal o HIGH (1) or LOW (0)
		if(HIGH_LOW==0)
			GPIOE->BSRR = GPIO_BSRR_BR3;
		else if (HIGH_LOW==1)
			GPIOE->BSRR = GPIO_BSRR_BS3;
	}	
void ACC_Init(){
	int data[10];
	
	//FIFO_CTRL bits 7-5 010 enables stream mode, which is 0x40
		data[0]= SPI_Send(0x2E | WRITE);
		
	
	
	
	
	
	
	
}


void ACC_Read(void){//put into a timer interrupt
	int status[10];
	int xHI[10];
	int yHI[10];
	int zHI[10];
	data2[0]= SPI_Send(0x20 | READ );
	
	setCS(0);
	status[0]= SPI_Send(0x27 | READ);//get stats
	data2[1] = SPI_Send(0x0); // dummy byte - to receive 16 bit from peripheral
	status[0] &=	~(0xF7) << (00001000);
	
	if((status[0]) == 8u){ //if status reg has bit 3 high then do
		//read OUT_X
			xHI[0]=SPI_Send(0x29 | READ);
		data2[1] = SPI_Send(0x0); // dummy byte - to receive 16 bit from peripheral
		//read OUT_Y
			yHI[0]=SPI_Send(0x2B | READ);
		data2[1] = SPI_Send(0x0); // dummy byte - to receive 16 bit from peripheral
		//read out_Z
			zHI[0]=SPI_Send(0x2D | READ);
		data2[1] = SPI_Send(0x0); // dummy byte - to receive 16 bit from peripheral
	}
	setCS(1);

}




