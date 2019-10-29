//TM1637 4 digit LED board


#ifndef TM1637_H_
#define TM1637_H_


#include <avr/io.h>
#include <stdint.h>
#include <stdlib.h>
#include <util/delay.h> //F_CPU="20000000" or 20MHz clock


class TM1637 
{
public:
	// function prototypes
	//void tm1637_def_pins();
	void init();
	void send_chars(uint8_t a_digit, uint8_t b_digit, uint8_t c_digit, uint8_t d_digit);
	void send_number(uint16_t four_digit_number);
	


private:
	
	//setup pins
	//data display
	#define tm1637_data_port PORTA
	#define tm1637_data_pin PIN1_bm //PA1
	
	#define tm1637_clk_port PORTA
	#define tm1637_clk_pin PIN2_bm //PA2
	
	
	// function prototypes
	void start();
	void stop();
	uint8_t data_out(uint8_t value);
	void tm1637_brightness(uint8_t level);
	
	
	// setup hex led segments
	//      A
	//     ---
	//  F |   | B
	//     -G-
	//  E |   | C
	//     ---
	//      D     X (dot)
	const uint8_t digitToSegment[16] = {
		// XGFEDCBA
		0b00111111,    // 0
		0b00000110,    // 1
		0b01011011,    // 2
		0b01001111,    // 3
		0b01100110,    // 4
		0b01101101,    // 5
		0b01111101,    // 6
		0b00000111,    // 7
		0b01111111,    // 8
		0b01101111,    // 9
		0b01110111,    // A
		0b01111100,    // b
		0b00111001,    // C
		0b01011110,    // d
		0b01111001,    // E
		0b01110001     // F
	};
	
	volatile uint8_t a, b, c, d;
	
};


#endif


