/*
 * BCD_encoder.h
 *
 * Created: 06/04/2019 15:41:48
 *  Author: david
 */ 


#ifndef BCD_ENCODER_H_
#define BCD_ENCODER_H_


#include <avr/io.h>
#include <stdint.h>
#include <stdlib.h>
#include <util/delay.h> //F_CPU="20000000" or 20MHz clock


class bcd_encoder
{
public:

	// function prototypes
	void init();
	uint16_t bcd_to_int();
		
	

private:
	
	#define bcd_bits 16 //number of bits to read out
	
	//setup pins
	//input BCD
	#define bcd_clock_port PORTB
	#define bcd_clock_pin PIN0_bm //PB0 - bcd clock
	
	#define bcd_data_port PORTB
	#define bcd_data_pin PIN1_bm //PB1 - bcd serial data - x4 bcd encoders (4bit)
	
	#define bcd_load_port PORTB
	#define bcd_load_pin PIN3_bm //PB3 - bcd read trigger
	
	
	// function prototypes
	uint16_t read_bcd();
	

};



#endif /* BCD_ENCODER_H_ */