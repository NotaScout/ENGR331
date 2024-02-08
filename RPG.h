
//singnal connectd to pins, on GPIOD
#define A 8
#define B 9
#define BUT 10

extern int DUTY;

void RPG_port_init(void);//sets up the pins
void Signal_A_interrupt_init(void);//sets signal A to be the interrupt
void RPG_Button_init(void);//sets up the button to be an interrupt, NOT PRESSED== 1
int DUTY_Increment(void);// sets custom DUTY incrementations // set to 10% of the period for best result




