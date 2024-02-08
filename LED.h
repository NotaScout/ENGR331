// LEDS are on PORT D PIN:  
#define LED_GREEN 12 
#define LED_ORANGE 13  
#define LED_RED 14 
#define LED_BLUE 15 

void LED_init(void);

void LED_ORANGE_enable(void);
void LED_BLUE_enable(void);
void LED_GREEN_enable(void);
void LED_RED_enable(void);

void LED_ON(int LED);
void LED_OFF(int LED);
void LED_toggle(int LED);


