//int clock for timer6
#include "stm32f4xx.h" 
#define receive 1 //gpioa
#define pin6 6 //gpioa



extern float TIME; //how long the sensor sends a high singal. 
//extern means that it is created elsewhere, lets you use it in main for ex. 
//it makes the varibale connected
extern float DIST; //distance messured from ultrasonic sensor
extern int i;

// initializations for timers
void TIM6_init(uint32_t  PSC_num, uint32_t  ARR_num);
void TIM4_init(uint32_t DUTY_num, uint32_t PERIOD);
void TIM2_init(uint32_t DUTY_num, uint32_t  ARR_num);
void TIM3_init(uint32_t DUTY_num, uint32_t  ARR_num);
// changing internal things for timers

//enables interrupt for timers
void TIM6_interrupt_int(void);





