 /* LCD.c
 * The goal of this lab is help you figure out
 * (on your own) how to interface to a peripheral
 * that you have never used before based on the
 * documentation you find (either datasheet or other
 * available sources/examples online).
 * ENJOY!!! (and I really mean it!)
 */

/*

TODO:
 - make code more readable & modular
 - use pinsets for set/clr functionality
 - figure out how to write chars (use the function templates)
 - ...


*/




#include "stm32f4xx.h" 
#include <string.h>
#include <stdio.h>
#include "LCD.h"


/*******************************
 * FUNCTION PROTOTYPES
 *******************************
 */
 /*
// LCD related functions; //
*/

void LCD_port_init(void);
void LCD_init(void);
void LCD_sendData(unsigned char data);
void LCD_sendInstr(unsigned char Instruction);
void LCD_place_cursor(unsigned char lineno);

/*
//void nibble(unsigned char nib);

// PIN set and clear functions
void PIN_clear(int PINNO);
void PIN_set(int PINNO);
void BF_check(void);
void PIN_MODE(int PINNO);

// END Functions
*/






/*******************************
 * START OF ACTUAL CODE
 *******************************
 */



void LCD_clear(void){
LCD_sendInstr(0);
LCD_sendInstr(1);



}

/*************************
 * FUNCTION DECLARATIONS:
 * LCD related functions
**************************/

/*******************************
 * LCD_port_init()
 * Inputs: NONE
 * Outputs: NONE
 * Port Initialization
 * Refer to the #define statements at top to
 * see what ports are used to connect
 * the STMicro Board with the HD44780 LCD driver
 * Set appropriate pins as digital input/outputs
 ******************
 * VERY IMPORTANT:  
 *****************
 * The Data7 (DB7) port (connected to GPIO PORT D) 
 * needs to be able to switch between
 * default mode OUTPUT (to send data) 
 * and INPUT (to check Busy Flag).
 *******************************
 */
void LCD_port_init(){
//STEP 1: Enable GPIOD in RCC AHB1ENR register
	RCC->AHB1ENR |= (1u<<3);

//STEP 2: Set MODER of GPIOD Pins 7, 6, 5, 3, 2, 1 & 0 as outputs
	GPIOD -> MODER &= ~(3u<<2*DB4); //2* for two bits
	GPIOD -> MODER |= (1u<<2*DB4);
	GPIOD -> MODER &= ~(3u<<2*DB5);
	GPIOD -> MODER |= (1u<<2*DB5);
	GPIOD -> MODER &= ~(3u<<2*DB6);
	GPIOD -> MODER |= (1u<<2*DB6);
	GPIOD -> MODER &= ~(3u<<2*DB7);
	GPIOD -> MODER |= (1u<<2*DB7);
	GPIOD -> MODER &= ~(3u<<2*EN);
	GPIOD -> MODER |= (1u<<2*EN);
	GPIOD -> MODER &= ~(3u<<2*RW);
	GPIOD -> MODER |= (1u<<2*RW);
	GPIOD -> MODER &= ~(3u<<2*RS);
	GPIOD -> MODER |= (1u<<2*RS);

//STEP 3: Set OTYPER of GPIOD Pins 7, 6, 5, 3, 2, 1 & 0 as push-pull
	/*GPIOD -> MODER &= ~(3u<<2*0); //2* for two bits
	GPIOD -> MODER &= ~(3u<<2*1);
	GPIOD -> MODER &= ~(3u<<2*2);
	GPIOD -> MODER &= ~(3u<<2*3);
	GPIOD -> MODER &= ~(3u<<2*4);
	GPIOD -> MODER &= ~(3u<<2*5);
	GPIOD -> MODER &= ~(3u<<2*6);
	GPIOD -> MODER &= ~(3u<<2*7);*/
	GPIOD -> OTYPER &= ~(127u); // SETS all OTYPER pins to 0 // (push pull config)

 
//Done with LCD port Initialization



}

/*******************************
 * LCD_init()
 * Inputs: NONE
 * Outputs: NONE
 * LCD Initialization
 * Read the manual carefully
 * We are doing INITIALIZATION BY INSTRUCTION
 * Don't rush it.
 *******************************
 */

void LCD_init(){

// STEP 1: Wait for 100ms for power-on-reset to take effect
	int i;
	for(i=0;i<=200;i++){}

// OK - nothing needs to be done here. 
// Keep the board poweLED_RED. By the time your code is downloaded
// to flash and you are ready to start execution using the 
// debugger - 100ms will have passed

// STEP 2: Set RS pin LOW to send instructions
	GPIOD -> ODR &= ~(1u<<RS); // fix // odr 7 =  // ands the 7th odr pin with 0
// Send instructions using following format:
// Check BF; Set EN=HIGH; Send 4-bit instruction; Set EN=low;
	//BF_check();
		
		/*
		// what does this do????
	//LCD_sendInstr(3);
	//GPIOD -> MODER &= ~(3u<<2*EN); //changed MODER5 to zero // sUs recheck // zeros out EN
	//GPIOD -> MODER |= (1u<<2*EN); // why do we need to change io type here? we don't use it // sets EN to 01 // lcd doesnt write to EN, only reads //
		// what
	*/
		
// STEP 3a-3d: Set 4-bit mode (takes a total of 4 steps)
		LCD_sendInstr(3); // 0010 0000 // sets 4-bit mode // 
		//BF_check();
		LCD_sendInstr(3); // home return
		//BF_check();
		LCD_sendInstr(3); // entry mode set (increment by 1)
		//BF_check();
		LCD_sendInstr(2); // turn on display
		//BF_check();

// STEP 4: Set 2 line display -- treats 16 char as 2 lines
//			001DL NF** (DL 0: 4bits; N= 1: 2 lines; F=0 : 5x8 display
	LCD_sendInstr(2); // we already did this is step 3a // 0010 0100
	//BF_check();
// STEP 5: Set DISPLAY to OFF
	LCD_sendInstr(8);
	LCD_sendInstr(0);
	LCD_sendInstr(8);
	LCD_sendInstr(0);
	LCD_sendInstr(1);
	LCD_sendInstr(0);
	LCD_sendInstr(6);
	LCD_sendInstr(0);
	LCD_sendInstr(15);
	LCD_sendInstr(0);
	LCD_sendInstr(2);

	//BF_check();
// STEP 6: CLEAR DISPLAY
	//LCD_sendInstr(0);
	//BF_check();
// STEP 7: SET ENTRY MODE - Auto increment; no scrolling
	//LCD_sendInstr(8);
	//BF_check();
// STEP 8: Set Display to ON with Cursor and Blink.
	//LCD_sendInstr(15);
	//BF_check();
	
}

/*******************************
 * LCD_place_cursor()
 * Inputs: unsigned character
 * Outputs: NONE
 * sets Cursor position to
 * Line 1, character 1 (hex address 0x80)
 * or Line 2, character 1 (hex addres 0xC0)
 *
 *******************************
 */

void LCD_place_cursor(unsigned char lineno){
	// have to place as 2 lines: clean up in the future to only need 1 line. Could probably clean up the general insruction command as well
	PIN_clear(RS);
	PIN_clear(RW); // sets to DRAM write
	PIN_set(EN); 
	GPIOD -> ODR &= ~(0xF0u) >> 4; // CLEAR INPUT PINS for instruction
	GPIOD -> ODR |= (lineno & 0xF0u) >> 4; // LALLALALALLA
	PIN_clear(EN); // marks instruction complete
	BF_check();
	
	PIN_clear(RS);
	PIN_clear(RW); // sets to DRAM write
	PIN_set(EN); 
	GPIOD -> ODR &= ~(0xFu) << DB4; // CLEAR INPUT PINS for instruction
	GPIOD -> ODR |= (lineno & 0xFu) << DB4; // LALLALALALLA
	PIN_clear(EN); // marks instruction complete
	BF_check();
	
}



/*******************************
 * LCD_sendData()
 * Inputs: unsigned character data (8-bit)
 * Outputs: NONE
 * writes the character to LCD.
 * Since we are using 4-bit mode
 * this function will take the character (8-bit)
 * transmit upper 4 bits and then lower 4 bits.
 * make sure the RS, RW and EN signals are set to correct value
 * for each 4-bit. 
 * also make sure to check the BF
 *******************************
 */

void LCD_sendData(unsigned char data)
{
	PIN_set(RS);
	PIN_clear(RW); // wriet mode
	PIN_set(EN);
	GPIOD -> ODR &= ~(0xFu) << DB4; // clear VERY IMPORTANT
	GPIOD -> ODR |= (data & 0xF0u) >> 4; // LALLALALALLA // writes .... [xxxx] yyyy  to least signifigant 4 digits, being []
	PIN_clear(EN); // marks instruction complete
	PIN_set(EN);
	GPIOD -> ODR &= ~(0xFu) << DB4; // clear VERY IMPORTANT
	GPIOD -> ODR |= (data & 0xFu) << DB4; // LALLALALALLA
	PIN_clear(EN); // marks instruction complete
	BF_check();
	// repurpose for more efficient instruction writing
	

}

/*******************************
 * LCD_sendInstr()
 * Inputs: unsigned character INSTRUCTION (8-bit)
 * Outputs: NONE
 * Sends commands to LCD
 * We are using 4-bit mode but 
 * this function accepts (8-bit) character
 * as input. You can make the call on how to handle that.
 * make sure the RS, RW and EN signals are set to correct value
 * for each 4-bit part of instructions. 
 * also make sure to check the BF
 *******************************
 */
//Instructions = 3;

void LCD_sendInstr(unsigned char Instruction)
{
	
	
	
	PIN_clear(RW); //GPIOD -> ODR &= ~(1u) << RW; // Sets RW to 0, sets LCD to write mode
	PIN_clear(RS);
	PIN_set(EN); //GPIOD -> ODR |= 1u << EN; // Sets enable to 1, step 1 to data transfer
	GPIOD -> ODR &= ~(0xFu) << DB4; // HAVE TP CLEAR THE INPUT BEFORE INPUTTING NEW INSTR// OMFG WHY DID I NOT THINK OF THIS UNTIL NOW // WHY WHYWHYWHWYHWY
	GPIOD -> ODR |= (Instruction & 0xFu) << DB4; // the 0xF0u is 1111, this paiLED_RED with the or mask selects the 4 MSB. This added with << DB4 shifts the 4 MSB to DB4 to DB7
	PIN_clear(EN); //GPIOD -> ODR |= 0u << EN; // marks instruction complete
	//BF_check();
	//PIN_set(EN);
	
	//GPIOD -> ODR |= (Instruction & 0xFu) << DB4; // captures LSBs // double check this works captures 0x[1111] 0000 // in theory
	//PIN_clear(EN);  //GPIOD -> ODR |= 0u << EN; // marks instruction complete
	BF_check();

}


/*******************************
 * PIN_clear()
 * Inputs: an integer PIN NUMBER (e.g. RW, EN)
 * Outputs: NONE
 * CLEARS PIN in GPIOD to 0
 * Read the Reference manual carefully
 * you can use the BSRR register without masks
 * OR you can use the ODR register WITH &~ (AND-NOT) mask 
 * to clear ONE specified pin.
 *******************************
 */
void PIN_clear(int PINNO){
	// unused for now, will use in future to make more readable//
	GPIOD -> ODR &= ~(1u << PINNO); // pin # (use pnemonic) to 0
	
	
}

/*******************************
 * PIN_set()
 * Inputs: an integer PIN NUMBER (e.g. RW, EN)
 * Outputs: NONE
 * SETS PIN in GPIOD to 1
 * Read the Reference manual carefully
 * you can use the BSRR register without masks
 * OR you can use the ODR register WITH | (OR) mask 
 * to SET ONE specified pin.
 *******************************
 */
void PIN_set(int PINNO){
	GPIOD -> ODR |= (1u << PINNO); // PIN # (use pnemonic) set to 1
	
}

/*******************************
 * BF_check()
 * Inputs: NONE
 * Outputs: NONE
 * Checks BF flag on DB7 pin of LCD
 * and prevents code from moving ahead
 * if the BF flag is 1 (indicating LCD busy)
 *******************************
 */

void BF_check(){
		int i;

	//int i = 0;
	//for(i=0;i<2000;i++){}
	/// STEP 1: Clear RS (set RS=0) as reading flag is an instruction sets it to instruction
	PIN_clear(RS);	//GPIOD -> ODR &= ~(3u<<2*RS); 
	
	// STEP 2: set Data Pin 7 connected to GPIOD Pin 3 as input  
	// 		   (no pull-up or pull down setup needed here)
	//GPIOD -> MODER &= ~(3u<<2*DB7);
	//GPIOD -> MODER |= (1u<<2*DB7); 
	
	// STEP 3: Set RW = 1 to read the BF flag. sets it to read
	//GPIOD -> MODER &= ~(3u<<2*RW); // don't change moder
	//GPIOD -> MODER |= (1u<<2*RW); // why are we changing mode here
	PIN_set(RW);
	GPIOD -> MODER &= ~(3u<<2*DB7); // sets db7 to input so it can read
	
	// STEP 4: Set EN = 1 
	PIN_set(EN);
	//GPIOD -> MODER &= ~(3u<<2*EN);
	//GPIOD -> MODER |= (1u<<2*EN);
	// STEP 5: Read the BUSY FLAG on Pin 3 of GPIOD.
	//		   Wait here if BUSY and keep reading pin  
	//         until BF becomes 0 indicating NOT BUSY.

	
	for(i=0;i<=200;i++){}
	while((GPIOD -> IDR & (1u<<DB7))){} // & 3 is what bit to look at ||STUCK HERE

	// STEP 6: CLEAR EN =0
	//GPIOD -> MODER &= ~(3u<<2*EN);
		PIN_clear(EN);
	//STEP 7: CLEAR RW =0 
	//GPIOD -> MODER &= ~(3u<<2*RW);
		PIN_clear(RW);

	//STEP 8: Set Data Pin 7 connected to GPIOD Pin 3 as output
	GPIOD -> MODER &= ~(3u<<2*DB7);
	GPIOD -> MODER |= (1u<<2*DB7);//*/

	

}

void  LCD_send_String(char string[]){ // use this fcn to output charstrings to lcd
int i = 0;
	for(i=0;string[i]!='\0';i++){
		LCD_sendData(string[i]); // make into function
	}
}

void  LCD_send_intString(int ANS){ // possibly breaks the code // double check ln; sprintf(ansstr,"%d",ANS);
int i = 0;
	char ansstr[8] = ""; // empty char string 8 chars long
	sprintf(ansstr,"%d",ANS); // function to send numbers to print
	for(i=0;ansstr[i]!='\0';i++){
		LCD_sendData(ansstr[i]);
	}

}

void full_reset(){
	LCD_clear();
	LCD_place_cursor(0x80);
	
}



void  LCD_send_floatString(float ANS){ // possibly breaks the code // double check ln; sprintf(ansstr,"%d",ANS);
int i = 0;
	char ansstr[17] = ""; // empty char string 8 chars long
	sprintf(ansstr,"%f",(double)ANS); // function to send numbers to print
	for(i=0;ansstr[i]!='\0';i++){
		LCD_sendData(ansstr[i]);
	}
	
}


