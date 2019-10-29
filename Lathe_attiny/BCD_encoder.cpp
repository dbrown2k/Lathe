/*
 * BCD_encoder.cpp
 *
 * Created: 06/04/2019 15:42:04
 *  Author: david
 */ 


#include "BCD_encoder.h"

void bcd_encoder::init()
{
	
	bcd_clock_port.DIRSET = bcd_clock_pin; 
	bcd_clock_port.OUTCLR = bcd_clock_pin; //clk low
	
	bcd_load_port.DIRSET = bcd_load_pin;
	bcd_load_port.OUTSET = bcd_load_pin; //trigger high
	
}


uint16_t bcd_encoder::read_bcd()
{
	volatile uint16_t data = 0;
	
	//load data
	bcd_load_port.OUTCLR = bcd_load_pin;
	_delay_us(5);
	//latch input to start shift out
	bcd_load_port.OUTSET = bcd_load_pin;
	_delay_us(5);
	
	//clock out data
	for (uint8_t i = 0; i < bcd_bits; i++)
	{
		
		data = data | (((bcd_data_port.IN & bcd_data_pin) >> (bcd_data_pin-1)) << i );
		
		//clk high - shift data
		bcd_clock_port.OUTSET = bcd_clock_pin;
		_delay_us(5);
		//clk low
		bcd_clock_port.OUTCLR = bcd_clock_pin;
		_delay_us(5);
		
	}
	
	return data;
}

uint16_t bcd_encoder::bcd_to_int()
{
	uint16_t temp = read_bcd();
	
	uint16_t ones = ((temp & 0b0000000000001000) >> 3) | ((temp & 0b0000000000000100) >> 1) | ((temp & 0b0000000000000010) << 1) | ((temp & 0b0000000000000001) << 3);
	uint16_t tens = ((temp & 0b0000000010000000) >> 7) | ((temp & 0b0000000001000000) >> 5) | ((temp & 0b0000000000100000) >> 3) | ((temp & 0b0000000000010000) >> 1);
	uint16_t hundereds = ((temp & 0b0000100000000000) >> 11) | ((temp & 0b0000010000000000) >> 9) | ((temp & 0b0000001000000000) >> 7) | ((temp & 0b0000000100000000) >> 5);
	uint16_t thousands = ((temp & 0b1000000000000000) >> 15) | ((temp & 0b0100000000000000) >> 13) | ((temp & 0b0010000000000000) >> 11) | ((temp & 0b0001000000000000) >> 9);
	
	temp = ones + (tens * 10) + (hundereds * 100) + (thousands * 1000);
	
	return temp;
}