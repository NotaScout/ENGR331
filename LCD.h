
/*******************************
 * LCD pins connections to PORTD
 *******************************
 */

#define RS 7
#define RW 6 
#define EN 5

#define DB7 3
#define DB6 2
#define DB5 1
#define DB4 0



void LCD_port_init(void);
void LCD_init(void);
void LCD_sendData(unsigned char data);
void LCD_sendInstr(unsigned char Instruction);
void LCD_place_cursor(unsigned char lineno);//0x80 for 1st line and 0xC0 for 2nd line. Now sends a bit
// write these
void LCD_clear(void); //
void LCD_send_character(char input);
void LCD_send_int(int integer);
void LCD_send_float(float floating_point_num);

// PIN set and clear functions
void PIN_clear(int PINNO);
void PIN_set(int PINNO);
void BF_check(void);
void PIN_MODE(int PINNO);


// Simplification Routines

void LCD_send_String(char inputstring[]);
void LCD_send_intString(int integer_number);
void LCD_send_floatString(float float_number);

void full_reset(void);




