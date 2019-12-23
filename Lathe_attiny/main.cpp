/*
 * Lathe_attiny.cpp
 *
 * Created: 29/03/2019 17:01:24
 * Author : david
 */ 

#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h> 
#include <avr/interrupt.h>
#include "TM1637.h"
#include "BCD_encoder.h"

//function prototypes
void cpu_clock_init();
void init_ADC();
void init_timers();
uint16_t get_adc(uint8_t sel_pin); //sel_pin = ADC_V or ADC_I
void variable_delay(uint16_t delay);
uint16_t rpm_out(uint16_t per_size);
void ramp_spindle_up();
void ramp_spindle_down();

//Global values
#define cpu_speed (20000000 * 60) // cycles per min / 20MHz * 60s - F_CPU="20000000"
#define spindle_steps 1600 // 1/8th step
#define min_spindle_driver_pulse_delay 100 //micro seconds (us)

#define feed_steps (200 * 16) // 1/8th step, 3200 step per rotation
#define feed_motor_teeth 21
#define feed_sprocket_teeth 60
#define feed_mm_per_rotation 1.8702857 //7/16 OD, 14 TPI lead screw - adjust by +0.056mm/rev calibration (for pulley variation, thread etc.
#define feed_mm_per_step (float)(feed_mm_per_rotation / ((float)feed_steps * ((float)feed_sprocket_teeth / (float)feed_motor_teeth)))
#define feed_default_PER 800

volatile uint16_t ramp_loops = -1;
#define ramp_acceleration 10 //ramp acceleration rpm / s / s

#define max_rpm 1200 //max rpm - change the number of steps for the sub 100rpm
#define adc_block_size 6 //makes control easier for the potentiometer
#define sub_100_steps 3 //number of steps per increment when rpm is less than 100

//Global variables
volatile uint8_t analogue_value = 0;
volatile uint8_t run_spindle = 0;
volatile uint8_t dir_spindle = 0;
volatile uint8_t run_feed = 0;
volatile uint8_t interlock = 1; //on startup neither should run until after both are off
volatile uint16_t spindle_speed = 0; //ideal rpm
volatile uint16_t spindle_ramp_counter = 0;
volatile uint8_t feed_ramp_counter = 0;
volatile uint16_t feed_rate = 0; // mm per revolution of main spindle
volatile uint8_t spindle_clock_divider = 0;
//const uint8_t divider_1    = 0b00000000; // 1/1 
//const uint8_t divider_2    = 0b00000010; // 1/2 
//const uint8_t divider_4    = 0b00000100; // 1/4 
//const uint8_t divider_8    = 0b00000110; // 1/8 
//const uint8_t divider_16   = 0b00001000; // 1/16 
//const uint8_t divider_64   = 0b00001010; // 1/64 
//const uint8_t divider_256  = 0b00001100; // 1/256 
//const uint8_t divider_1024 = 0b00001110; // 1/1024 
#define bins 4
const uint16_t spindle_clock_divider_array[bins] = {16, 64, 256, 1024}; //{1, 2, 4, 8, 16, 64, 256, 1024} only using subset as this gets us the required value
const uint8_t divider_array[bins] = {0b00001000 ,0b00001010, 0b00001100, 0b00001110}; //{0b00000000, 0b00000010, 0b00000100, 0b00000110, 0b00001000 ,0b00001010, 0b00001100, 0b00001110}


//inputs
#define analogue_main_mask PIN3_bm //PA3 - spindle speed

//PA4 - feed direction
#define feed_direction_switch_port PORTA
#define feed_direction_switch_mask PIN4_bm 
#define feed_direction_switch_control PORTA.PIN4CTRL
#define feed_direction_switch_pin_position PIN4_bp

//PA7 - spindle direction
#define spindle_direction_switch_port PORTA
#define spindle_direction_switch_mask PIN7_bm 
#define spindle_direction_switch_control PORTA.PIN7CTRL
#define spindle_direction_switch_pin_position PIN7_bp

//PA6 - spindle enable - ext interrupt
#define spindle_enable_switch_port PORTA
#define spindle_enable_switch_mask PIN6_bm 
#define spindle_enable_switch_pin_position PIN6_bp
#define spindle_enable_switch_control PORTA.PIN6CTRL 

//PB2 - feed enable - ext interrupt
#define feed_enable_switch_port PORTB
#define feed_enable_switch_mask PIN2_bm 
#define feed_enable_switch_pin_position PIN2_bp
#define feed_enable_switch_control PORTB.PIN2CTRL


//control outputs
//PC1 - enable spindle - active low
#define spindle_enable_port PORTC
#define spindle_enable_mask PIN1_bm 
#define spindle_enable_control PORTC.PIN1CTRL

//PC2 - enable feed - active low
#define feed_enable_port PORTC
#define feed_enable_mask PIN2_bm 
#define feed_enable_control PORTC.PIN2CTRL

//PC3 - step spindle - timer counter A (TCA0)
#define step_spindle_port PORTC
#define step_spindle_mask PIN3_bm
#define step_spindle_control PORTC.PIN3CTRL

//PC0 - step feed - timer counter B (TCB0)
#define step_feed_port PORTC
#define step_feed_mask PIN0_bm
#define step_feed_control PORTC.PIN0CTRL


//instantiate class/structure instances
TM1637 led; //init LED display
bcd_encoder bcd_input; //init BCD input encoder


//interrupt for TCA0 split high underflow - spindle pin
ISR (TCA0_HUNF_vect)
{
	//TCA0.SPLIT.INTFLAGS  //clear interrupt flag
	if (spindle_ramp_counter == 0)
	{
		TCA0.SPLIT.INTCTRL &= ~(TCA_SPLIT_HUNF_bm); //disable interrupt flag)
	}
	else
	{
		spindle_ramp_counter--;
	}
	
}

//interrupt for TCB0 - toggle pin
ISR (TCB0_INT_vect)
{
	//disable interrupts whilst clock updates
	cli();
	TCB0.INTFLAGS = TCB_CAPT_bm;  //Clear the interrupt flag 
	step_feed_port.OUTTGL = step_feed_mask;  //Toggle step GPIO 
	
	//decrement ramp up counter
	if (feed_ramp_counter > 0)
	{
		feed_ramp_counter--;
	}
	//enable interrupts once clock updated
	sei();
}

//setup interrupt for changes on enable pins
ISR (PORTA_PORT_vect)
{
	//disable interrupts whilst clock updates
	cli();
	spindle_enable_switch_port.INTFLAGS = PORT_INT_gm; //read pins and update flags (clear interrupt)
	dir_spindle = ((spindle_direction_switch_port.IN & spindle_direction_switch_mask) >> spindle_direction_switch_pin_position);
	run_spindle = ((spindle_enable_switch_port.IN & spindle_enable_switch_mask) >> spindle_enable_switch_pin_position) | dir_spindle;			
	run_feed = ((feed_direction_switch_port.IN & feed_direction_switch_mask) >> feed_direction_switch_pin_position);
	//enable interrupts once clock updated
	sei();
}

//setup interrupt for changes on enable pins
ISR (PORTB_PORT_vect)
{
	//disable interrupts whilst clock updates
	cli();
	feed_enable_switch_port.INTFLAGS = PORT_INT_gm; //read pins and update flags (clear interrupt)
	run_feed = ((feed_enable_switch_port.IN & feed_enable_switch_mask) >> feed_enable_switch_pin_position);
	//enable interrupts once clock updated
	sei();
}	



///////////////////////////////////////////////////////////////////////////////
///////////////////////// START of MAIN ///////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
int main(void)
{
	//set CPU to run at full speed (default is 3.3MHz)
	cpu_clock_init();
	
	//set input, pull-ups on pins, interrupt from pins, etc
	spindle_enable_switch_control =		PORT_ISC_BOTHEDGES_gc | PORT_PULLUPEN_bm | PORT_INVEN_bm; 
	spindle_direction_switch_control =	PORT_ISC_BOTHEDGES_gc | PORT_PULLUPEN_bm | PORT_INVEN_bm;
	feed_enable_switch_control =		PORT_ISC_BOTHEDGES_gc | PORT_PULLUPEN_bm | PORT_INVEN_bm;
	feed_direction_switch_control =		PORT_ISC_BOTHEDGES_gc | PORT_PULLUPEN_bm | PORT_INVEN_bm;

	//set outputs
	step_spindle_port.DIRSET = step_spindle_mask;
	step_spindle_port.OUTCLR = step_spindle_mask;
	PORTMUX.CTRLC = PORTMUX_TCA03_ALTERNATE_gc; //TCA03 alt pin
	
	step_feed_port.DIRSET = step_feed_mask;
	step_feed_port.OUTCLR = step_feed_mask;
	PORTMUX.CTRLD = PORTMUX_TCB0_ALTERNATE_gc; //TCB0 alt pin
	
	
	spindle_enable_control = PORT_INVEN_bm;
	spindle_enable_port.DIRSET = spindle_enable_mask;
	spindle_enable_port.OUTCLR = spindle_enable_mask;
	
	feed_enable_control = PORT_INVEN_bm;
	feed_enable_port.DIRSET = feed_enable_mask;
	feed_enable_port.OUTCLR = feed_enable_mask;
	
	
	//initialise 
	init_ADC();
	led.init();
	bcd_input.init();
	
	
	//on startup if any switches (enable or direction are enabled, halt and wait for safe state
	if (interlock == 1)
	{
		while ( (spindle_enable_switch_port.IN & spindle_enable_switch_mask) >= 1 
		|| (spindle_direction_switch_port.IN & spindle_direction_switch_mask) >= 1
		|| (feed_enable_switch_port.IN & feed_enable_switch_mask) >= 1
		|| (feed_direction_switch_port.IN & feed_direction_switch_mask) >= 1 )
		
		{
				//wait
		}
		
		interlock = 0;
		
		//position lock the spindle
		spindle_enable_port.OUTSET = spindle_enable_mask;
		
		for (uint8_t i = 0; i <= 6; i++)
		{
			step_spindle_port.OUTTGL = step_spindle_mask;
			_delay_ms(min_spindle_driver_pulse_delay);
		}
		
		step_spindle_port.OUTCLR = step_spindle_mask;
		spindle_enable_port.OUTCLR = spindle_enable_mask;
	}
	
	
	//can start timers after all safe
	init_timers();
	
	//enable interrupts
	sei();
	
    while (1) 
    {
		//asm volatile ("nop");
		
		//////////////////////////////////////////////////////////////////////////
		//main loop
		
		//while waiting for startup
		while (interlock == 0 && run_spindle == 0 && run_feed == 0)
		{
			//disable timers
			TCA0.SPLIT.CTRLA &= ~TCA_SPLIT_ENABLE_bm;
			TCB0.CTRLA &= ~TCB_ENABLE_bm;
			
			//scale main clock - default
			spindle_clock_divider = bins-1; //div_max
			TCA0.SPLIT.CTRLA |= divider_array[spindle_clock_divider];
			TCA0.SPLIT.CTRLB = TCA_SPLIT_HCMP0EN_bm; //enable timer interrupt
			//setup safe - lowest frequency
			TCA0.SPLIT.HPER = 255;
			TCA0.SPLIT.HCMP0 = 128;
			
			//set the ramp counters to max
			spindle_ramp_counter = -1;
			feed_ramp_counter = -1;
			
			
			while (interlock == 0 && run_spindle == 0 && run_feed == 0)
			{
				feed_rate = bcd_input.bcd_to_int(); //read BCD counter
				analogue_value = (255 - get_adc(3)); //read potentiometer
				
				//set analogue value based on divider
				analogue_value = (analogue_value / adc_block_size) + 1;
				
				//calculate ideal RPM from analogue value
				if ((uint16_t)analogue_value < (((uint16_t)max_rpm - 100) / 100))
				{
					spindle_speed = (uint16_t)((uint16_t)max_rpm - ((uint16_t)analogue_value * 100));
				} 
				else
				{
					spindle_speed =  (uint16_t)(100 - ((uint16_t)analogue_value - ((((uint16_t)max_rpm - 100) / 100) + 1)) * sub_100_steps);
				}
				
				//calculate closest value clock settings to achieve requested / ideal rpm
				uint8_t temp_integer = 0;
				uint16_t temp_value = -1; //max value
						
				//calculate for each clock divider and keep the one that's closest to the desired rpm
				for (uint8_t n = 0; n < bins; n++)
				{
					//for each clock divider calculate the integer that gives rpm closest to requested
					uint8_t temp_div_integer = (uint8_t)((float)cpu_speed / ((float)spindle_clock_divider_array[n] * (float)spindle_speed * (float)spindle_steps));
					//calculate the rpm values to allow calculation of the closest value
					int16_t temp_div_rpm = (int16_t)(((float)cpu_speed / ((float)spindle_clock_divider_array[n]*float(temp_div_integer)*(float)spindle_steps)));
					//compare values to current best value
					if ((uint16_t)abs((int16_t)spindle_speed - temp_div_rpm) < temp_value)
					{
						temp_value = abs(spindle_speed - temp_div_rpm);
						temp_integer = temp_div_integer;
						spindle_clock_divider = n;
					}
				}
				
				//update analogue value to nearest value for the selected clock divider
				analogue_value = temp_integer;
				
				ramp_loops = (uint16_t)(ramp_acceleration * ((float)cpu_speed / ((float)spindle_clock_divider_array[spindle_clock_divider] * analogue_value * 1200)));
				
				//update the clock divider.
				cli();
				TCA0.SPLIT.CTRLA = 0;
				TCA0.SPLIT.CTRLA |= divider_array[spindle_clock_divider];
				sei();
				
				
				led.send_number(rpm_out(analogue_value)); //sends the rpm
			}
			
		}
		
		//////////////////////////////////////////////////////////////////////////
		//spindle only
		while ( (interlock == 0 && run_spindle == 1 && run_feed == 0 && dir_spindle == 0) || (interlock == 0 && run_spindle == 1 && run_feed == 0 && dir_spindle == 1 && spindle_speed <= 20) )
		{
			_delay_ms(min_spindle_driver_pulse_delay);
			
			uint8_t old_dir_spindle = dir_spindle;
			
			//check for previous running
			if (spindle_ramp_counter != 0)
			{				
				//enable timers
				TCA0.SPLIT.CTRLA |= TCB_ENABLE_bm;
				
				//enable spindle motor
				spindle_enable_port.OUTSET = spindle_enable_mask;
				_delay_us(min_spindle_driver_pulse_delay); //delay to allow init of driver
				
				//ramp up
				ramp_spindle_up();	
			}
					
			while (interlock == 0 && run_spindle == 1 && run_feed == 0)
			{
				//hold here until interrupt
			}
			
			//if spindle off after interrupt, then ramp down the spindle
			if (run_spindle == 0 && old_dir_spindle == 0)
			{
				//ramp speed down
				ramp_spindle_down();
				
				//disable spindle
				spindle_enable_port.OUTCLR = spindle_enable_mask;
			}
			
			
		}
		
		//////////////////////////////////////////////////////////////////////////
		//feed only
		while (interlock == 0 && run_spindle == 0 && run_feed == 1)
		{
			_delay_ms(min_spindle_driver_pulse_delay);
			
			//set feed rate
			TCB0.CCMP = feed_default_PER; //set pulse length
			
			//enable timers
			TCB0.CTRLA |= TCB_ENABLE_bm;
			
			//enable feed motor
			feed_enable_port.OUTSET = feed_enable_mask;
			_delay_us(min_spindle_driver_pulse_delay); //delay for allow driver startup
			
			while (interlock == 0 && run_spindle == 0 && run_feed == 1)
			{
				//hold here until interrupt
			}
			
			//disable feed motor
			feed_enable_port.OUTCLR = feed_enable_mask;
		}
		
		//////////////////////////////////////////////////////////////////////////
		//standard spindle with feed - feed rate number is arbitrary value
		//spindle must be started / running before entering this mode
		while (interlock == 0 && run_spindle == 1 && spindle_ramp_counter == 0 && run_feed == 1 && spindle_speed > 20)
		{
			_delay_ms(min_spindle_driver_pulse_delay);
			
			//set feed rate
			TCB0.CCMP = feed_default_PER*20; //set pulse length
			
			//enable feed timer
			TCB0.CTRLA |= TCB_ENABLE_bm; //enable clock	
			
			//enable feed motor
			feed_enable_port.OUTSET = feed_enable_mask;
			_delay_us(min_spindle_driver_pulse_delay); //delay for allow driver startup
	
			
			while (interlock == 0 && run_spindle == 1 && spindle_ramp_counter == 0 && run_feed == 1 && spindle_speed > 20)
			{
				//hold here until interrupt
			}
			
			//disable feed motor
			feed_enable_port.OUTCLR = feed_enable_mask;
			
			//if spindle off after interrupt, then ramp down the spindle
			if (run_spindle == 0)
			{
				ramp_spindle_down();
				
				_delay_ms(100); //delay to allow init of driver
				
				//disable spindle
				spindle_enable_port.OUTCLR = spindle_enable_mask;
			}
		}
		
		//////////////////////////////////////////////////////////////////////////
		//Engaged spindle with feed / slow spindle
		while (interlock == 0 && run_spindle == 1 && run_feed == 1 && spindle_speed <= 20)
		{
			_delay_ms(min_spindle_driver_pulse_delay);
			
			//disable timers
			TCA0.SPLIT.CTRLA &= ~TCA_SPLIT_ENABLE_bm;
			TCB0.CTRLA &= ~TCB_ENABLE_bm;	
			TCA0.SPLIT.CTRLB = 0; //disable timer interrupt		
			
			//calculate stepping ratio
			//convert feed_rate to the number of steps the spindle makes per step the feed makes
			float spindle_steps_per_feed_step = (float)spindle_steps / ((float)feed_rate / feed_mm_per_step / 1000);
			
			volatile float current_decimal = spindle_steps_per_feed_step;
			volatile uint16_t current_spindle_steps = (uint16_t)spindle_steps_per_feed_step; 
			
			//enable spindle & feed motor
			spindle_enable_port.OUTSET = spindle_enable_mask;
			feed_enable_port.OUTSET = feed_enable_mask;
			_delay_us(min_spindle_driver_pulse_delay); //delay for allow driver startup
			
			while (interlock == 0 && run_spindle == 1 && run_feed == 1 && spindle_speed <= 20)
			{
				//hold in here until interrupt
				//evaluate next step
				if ((current_decimal + spindle_steps_per_feed_step) > 1)
				{
					current_decimal = current_decimal + spindle_steps_per_feed_step - current_spindle_steps;
				} 
				else
				{
					current_decimal = current_decimal + spindle_steps_per_feed_step;
				}
				
				//extract only the whole digits for spindle steps
				current_spindle_steps = (uint16_t)current_decimal;
				
				//loop to required number of spindle steps per feed step - this can be zero
				for (uint16_t i = current_spindle_steps; i > 0; i--)
				{
					step_spindle_port.OUTTGL = step_spindle_mask;  //Toggle step GPIO 
					_delay_us(min_spindle_driver_pulse_delay/2); //delay
					
					if (run_spindle == 0)
					{
						break;
					}
				}
				
				step_feed_port.OUTTGL = step_feed_mask;  //Toggle step GPIO 
				_delay_us(min_spindle_driver_pulse_delay/2); //delay
			}
			
			//disable spindle & feed
			spindle_enable_port.OUTCLR = spindle_enable_mask;
			feed_enable_port.OUTCLR = feed_enable_mask;
		}
    }
	
}


//setup the CPU full 20MHz
void cpu_clock_init(void) {
	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 0);
}


//Setup ADC
void init_ADC()
{
	//TINY - setup ADC
	//VREF - setup voltage reference to 4.3V
	VREF.CTRLA = VREF_ADC0REFSEL_4V34_gc;
	//VREF - enable ADC ref
	VREF.CTRLB |= VREF_ADC0REFEN_bm;
	//ADC - select resolution 8bit
	ADC0.CTRLA = ADC_RESSEL_8BIT_gc;
	//ADC - number of convertions accumulated per measurement
	ADC0.CTRLB = ADC_SAMPNUM_ACC8_gc;
	//ADC - select reference
	ADC0.CTRLC = ADC_REFSEL_INTREF_gc;
	//ADC - sampling rate pre-scaler ~1.25MHz
	ADC0.CTRLC = ADC_PRESC_DIV16_gc;
	//ADC - initial input PA3 / AIN3
	ADC0.MUXPOS = ADC_MUXPOS_AIN3_gc;
	//ADC - enable start event (start measuring on enable)
	ADC0.EVCTRL |= ADC_STARTEI_bm;
	//ADC - enable ADC (ready for measurement trigger)
	ADC0.CTRLA |= ADC_ENABLE_bm;
};

//setup - Counter A & B
void init_timers()
{
	//TCA0 - setup 8bit
	//enable split mode on TCA0 to allow signal to get to pin C3
	TCA0.SINGLE.CTRLD = TCA_SINGLE_SPLITM_bm;
	TCA0.SPLIT.HPER = 255; //longest wait for given clock
	TCA0.SPLIT.HCMP0 = 128;
	TCA0.SPLIT.HCNT = 0; 
	TCA0.SPLIT.CTRLB = TCA_SPLIT_HCMP0EN_bm;
	TCA0.SPLIT.DBGCTRL |= TCA_SPLIT_DBGRUN_bm;
	
	//scale main clock - default
	spindle_clock_divider = 5;
	TCA0.SPLIT.CTRLA |= divider_array[spindle_clock_divider];
	
	//TCB0 - setup 16bit
	TCB0.CCMP = 0; //set pulse length - high 

	//set 8bit PWM mode - this connects it to its pin PA5 (or alternate)
	TCB0.CTRLA = 0 << TCB_RUNSTDBY_bp /* Run Standby: disabled */
				| 0 << TCB_SYNCUPD_bp /* Synchronize Update: enabled */
				| TCB_CLKSEL_CLKDIV1_gc  /* use MAIN as clock source */
				| 0 << TCB_ENABLE_bp;   /* Enable: disabled */

	TCB0.CTRLB = 0 << TCB_ASYNC_bp      /* Asynchronous Enable: disabled */
				| 0 << TCB_CCMPINIT_bp /* Pin Initial State: enabled, used for input operations */
				| 0 << TCB_CCMPEN_bp   /* Pin Output Enable: enabled */
				| TCB_CNTMODE_INT_gc; /* Periodic Interrupt */
	
	TCB0.DBGCTRL |= TCB_DBGRUN_bm;
	
	TCB0.INTCTRL = TCB_CAPT_bm; //enable interrupt for clocking pin
}

//read ADC value
uint16_t get_adc(uint8_t sel_pin)
{
	//wait for current measurement to finish
	while((ADC0.COMMAND & ADC_STCONV_bm) == 1){}
	//select input PA2 / AIN2
	ADC0.MUXPOS = sel_pin;
	//trigger reading
	ADC0.COMMAND |= ADC_STCONV_bm;
	//wait while reading taken
	while((ADC0.COMMAND & ADC_STCONV_bm) == 1){}
	//process result
	uint8_t result = uint8_t(ADC0.RES/8); //divide by number of samples accumulated
	return result;
}

//update LED rpm from ADC
uint16_t rpm_out(uint16_t per_size)
{
	
	uint16_t out_number = uint16_t(((float)cpu_speed / ((float)spindle_clock_divider_array[spindle_clock_divider]*float(per_size)*(float)spindle_steps)));
	
	return out_number;
	
}


void ramp_spindle_up()
{
	//ramp the frequency
	for (uint8_t n = 255; n >= analogue_value; n--)
	{
		TCA0.SPLIT.HPER = n;
		TCA0.SPLIT.HCMP0 = n/2; //HPER divide by 2 ~50% duty cycle
		
		//calculate number of loops to delay to achieve required acceleration of xRPM / second
		//calculate current rpm
		//calculate next rpm
		//calculate difference
		//calculate steps to match required acceleration rpm/second 
		
		spindle_ramp_counter = ramp_loops;
		
		TCA0.SPLIT.INTCTRL |= TCA_SPLIT_HUNF_bm; //enable interrupt flag
		
		while (spindle_ramp_counter != 0 && run_spindle == 1)
		{
			//hold here until interrupt loop finished
			//delay equal to 
		}
		
		//if ramp up canceled then exit and disable
		if (run_spindle == 0)
		{
			//disable spindle
			spindle_enable_port.OUTCLR = spindle_enable_mask;
			
			//exit ramp up
			return;
		}
	}
}


void ramp_spindle_down()
{
	//ramp the frequency
	for (uint8_t n = analogue_value; n <= 64; n++)
	{
		TCA0.SPLIT.HPER = n;
		TCA0.SPLIT.HCMP0 = n/2; //HPER divide by 2 ~50% duty cycle
		
		spindle_ramp_counter = ramp_loops;
		
		TCA0.SPLIT.INTCTRL |= TCA_SPLIT_HUNF_bm; //enable interrupt flag
		
		while (spindle_ramp_counter != 0)
		{
			//hold here until interrupt loop finished
		}
	}
}


