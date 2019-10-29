//TM1637 4 digit LED board

#include "TM1637.h"


void TM1637::init()
{
	//TINY - setup port A (1 = output)
	// 7 6 5 4 3 2 1 0
	tm1637_data_port.DIRSET = tm1637_data_pin;
	tm1637_clk_port.DIRSET = tm1637_clk_pin;
	
	//set pins high
	tm1637_clk_port.OUTSET = tm1637_clk_pin;
	tm1637_data_port.OUTSET = tm1637_data_pin;
	
	TM1637::tm1637_brightness(6);
}

void TM1637::start()
{
	//set clk high
	tm1637_clk_port.OUTSET = tm1637_clk_pin;
	//set data high
	tm1637_data_port.OUTSET = tm1637_data_pin;
	//wait 5us
	_delay_us(10);
	
	//set data low
	tm1637_data_port.OUTCLR = tm1637_data_pin;
	//sel clk low
	tm1637_clk_port.OUTCLR = tm1637_clk_pin;
	//wait 5us
	_delay_us(10);
}



void TM1637::stop()
{
	//clk low
	tm1637_clk_port.OUTCLR = tm1637_clk_pin;
	//data low
	tm1637_data_port.OUTCLR = tm1637_data_pin;
	//wait 5us
	_delay_us(10);
	
	//clk high
	tm1637_clk_port.OUTSET = tm1637_clk_pin;
	//data high
	tm1637_data_port.OUTSET = tm1637_data_pin;
	//wait 5us
	_delay_us(10);
}


uint8_t TM1637::data_out(uint8_t value)
{
	//going to ignore receiving the ACK bit
	
	for (uint8_t i = 0; i < 8; i++)
	{
		//clk low
		tm1637_clk_port.OUTCLR = tm1637_clk_pin;
		_delay_us(10);
		//set data to match bit value
		if ( ((value & (1<<i)) >> i) == 1)
		{
			tm1637_data_port.OUTSET = tm1637_data_pin;
		} 
		else
		{	
			tm1637_data_port.OUTCLR = tm1637_data_pin;
		}
		_delay_us(10);
		//clk high
		tm1637_clk_port.OUTSET = tm1637_clk_pin;
		_delay_us(10);
	}
	
	//clk low
	tm1637_clk_port.OUTCLR = tm1637_clk_pin;
	_delay_us(10);
	
	//data input
	tm1637_data_port.DIRCLR = tm1637_data_pin;
	
	//clk high
	tm1637_clk_port.OUTSET = tm1637_clk_pin;
	_delay_us(10);
	
	//wait for ACK
	uint8_t ack = (~tm1637_data_port.IN & tm1637_data_pin) == 0;
	
	//data output
	tm1637_data_port.DIRSET = tm1637_data_pin;
	
	_delay_us(100);
	
	return ack;
}


void TM1637::tm1637_brightness(uint8_t level)
{
	TM1637::start();
	
	TM1637::data_out(0b10001000 | (0b00000111 & level));
	
	TM1637::stop();
}


void TM1637::send_chars(uint8_t a_digit, uint8_t b_digit, uint8_t c_digit, uint8_t d_digit)
{
	TM1637::start();
	
	TM1637::data_out(0b01000000);
	
	TM1637::stop();
	
	
	TM1637::start();
	
	TM1637::data_out(0b11000000);
	
	TM1637::data_out(a_digit);
	TM1637::data_out(b_digit);
	TM1637::data_out(c_digit);
	TM1637::data_out(d_digit);
	
	TM1637::stop();
}


void TM1637::send_number(uint16_t four_digit_number)
{
	if (four_digit_number > 999)
	{
		a = uint8_t(four_digit_number / 1000 % 10);
	} 
	else
	{
		a = 0;
	}
	
	if (four_digit_number > 99)
	{
		b = uint8_t(four_digit_number / 100 % 10);
	} 
	else
	{
		b = 0;
	}
	
	if (four_digit_number > 9)
	{
		c = uint8_t(four_digit_number / 10 % 10);
	} 
	else
	{
		c = 0;
	}
	
	if (four_digit_number >= 0)
	{
		d = uint8_t(four_digit_number % 10);
	} 
	else
	{
		d = 0;
	}
		
		
	TM1637::send_chars(digitToSegment[a], digitToSegment[b], digitToSegment[c], digitToSegment[d]);
		
}