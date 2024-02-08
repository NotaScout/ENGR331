#include "stm32f4xx.h" 

#define READ  0x080 //reads reg
#define WRITE 0x00 //write to reg
extern uint8_t data2[10];

void SPI_Port_Init(void);
void SPI_Init(void);
void ACC_Init(void);
void ACC_Read(void); // read accelerometer
