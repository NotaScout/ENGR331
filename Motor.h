//inputs to the H-bridge

#define Hbridge1A_4A 14
#define Hbridge2A_3A 13

// prototypes

void Motor_port_init(void);
void Motor_forward(void); // set 10
void Motor_backward(void); // set 01
void Motor_stop(void); // set 00



