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
void syncronise_encoder();


//Global values
#define CPU_SPEED (F_CPU * 60) // cycles per min
#define SPINDLE_STEPS 1600 // 1/8th step
#define MIN_SPINDLE_DRIVER_PULSE_DELAY 100 //micro seconds (us)

#define FEED_STEPS (200 * 16) // 1/16th step, 3200 step per rotation
#define FEED_MOTOR_TEETH 21
#define FEED_SPROCKET_TEETH 60
#define FEED_MM_PER_ROTATION 1.8702857 //7/16 OD, 14 TPI lead screw - adjust by +0.056mm/rev calibration (for pulley variation, thread etc.
#define FEED_MM_PER_STEP (float)(FEED_MM_PER_ROTATION / ((float)FEED_STEPS * ((float)FEED_SPROCKET_TEETH / (float)FEED_MOTOR_TEETH)))

#define FEED_STEP_DURATION 40 //micro seconds
#define FEED_DEFAULT_PER (FEED_STEP_DURATION * (F_CPU / 1000000)) //no. cycles per timer step
#define FEED_BCD_DIVISOR 1000 //BCD counter - 1/1000ths of a millimeter

volatile uint16_t ramp_loops = -1; //-1 unsigned = max value
#define RAMP_ACCELERATION 10 //ramp acceleration rpm / s / s

#define MAX_RPM 1200 //max rpm
#define FINE_CONTROL_RPM 100 //increase resolution below this rpm
#define STANDARD_CONTROL_STEPS 6 //makes control easier for the potentiometer
#define FINE_CONTROL_STEPS 3 //number of steps per increment when rpm is less than fine control value

//Global variables
volatile uint8_t analogue_value = 0;
volatile uint8_t run_spindle = 0;
volatile uint8_t dir_spindle = 0;
volatile uint8_t run_feed = 0;
volatile uint8_t interlock = 1; //on startup neither should run until after both are off
volatile uint16_t spindle_speed = 0; //ideal rpm
volatile uint16_t spindle_ramp_counter = 0;
volatile uint8_t spindle_running = 0;
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
#define BINS 4 //number of subsets of clock divider to get required values
const uint16_t spindle_clock_divider_array[BINS] = {16, 64, 256, 1024}; //{1, 2, 4, 8, 16, 64, 256, 1024} only using subset as this gets us the required values
const uint8_t divider_array[BINS] = {0b00001000 ,0b00001010, 0b00001100, 0b00001110}; //{0b00000000, 0b00000010, 0b00000100, 0b00000110, 0b00001000 ,0b00001010, 0b00001100, 0b00001110}


//inputs
#define ANALOGUE_MAIN_MASK PIN3_bm //PA3 - spindle speed potentiometer

//PA4 - feed direction
#define FEED_DIRECTION_SWITCH_PORT PORTA
#define FEED_DIRECTION_SWITCH_MASK PIN4_bm 
#define FEED_DIRECTION_SWITCH_CONTROL PORTA.PIN4CTRL
#define FEED_DIRECTION_SWITCH_PIN_POSITION PIN4_bp

//PA7 - spindle direction
#define SPINDLE_DIRECTION_SWITCH_PORT PORTA
#define SPINDLE_DIRECTION_SWITCH_MASK PIN7_bm 
#define SPINDLE_DIRECTION_SWITCH_CONTROL PORTA.PIN7CTRL
#define SPINDLE_DIRECTION_SWITCH_PIN_POSITION PIN7_bp

//PA6 - spindle enable - ext interrupt
#define SPINDLE_ENABLE_SWITCH_PORT PORTA
#define SPINDLE_ENABLE_SWITCH_MASK PIN6_bm 
#define SPINDLE_ENABLE_SWITCH_PIN_POSITION PIN6_bp
#define SPINDLE_ENABLE_SWITCH_CONTROL PORTA.PIN6CTRL 

//PB2 - feed enable - ext interrupt
#define FEED_ENABLE_SWITCH_PORT PORTB
#define FEED_ENABLE_SWITCH_MASK PIN2_bm 
#define FEED_ENABLE_SWITCH_PIN_POSITION PIN2_bp
#define FEED_ENABLE_SWITCH_CONTROL PORTB.PIN2CTRL


//control outputs
//PC1 - enable spindle - active low
#define SPINDLE_ENABLE_PORT PORTC
#define SPINDLE_ENABLE_MASK PIN1_bm 
#define SPINDLE_ENABLE_CONTROL PORTC.PIN1CTRL

//PC2 - enable feed - active low
#define FEED_ENABLE_PORT PORTC
#define FEED_ENABLE_MASK PIN2_bm 
#define FEED_ENABLE_CONTROL PORTC.PIN2CTRL

//PC3 - step spindle - timer counter A (TCA0)
#define STEP_SPINDLE_PORT PORTC
#define STEP_SPINDLE_MASK PIN3_bm
#define STEP_SPINDLE_CONTROL PORTC.PIN3CTRL

//PC0 - step feed - timer counter B (TCB0)
#define STEP_FEED_PORT PORTC
#define STEP_FEED_MASK PIN0_bm
#define STEP_FEED_CONTROL PORTC.PIN0CTRL


//instantiate class/structure instances
TM1637 led; //init LED display
bcd_encoder bcd_input; //init BCD input encoder


//interrupt for TCA0 split high underflow - spindle pin
ISR (TCA0_HUNF_vect)
{
	//TCA0.SPLIT.INTFLAGS  //clear interrupt flag
	if (spindle_ramp_counter == 0)
	{
		TCA0.SPLIT.INTCTRL &= ~(TCA_SPLIT_HUNF_bm); //disable interrupt
	}
	else
	{
		spindle_ramp_counter--;
	}
	
}

//interrupt for TCB0 - toggle pin
ISR (TCB0_INT_vect)
{
	TCB0.INTFLAGS = TCB_CAPT_bm;  //Clear the interrupt flag 
	STEP_FEED_PORT.OUTTGL = STEP_FEED_MASK;  //Toggle step GPIO 
	
	//decrement ramp up counter
	if (feed_ramp_counter > 0)
	{
		feed_ramp_counter--;
	}
}

//setup interrupt for changes on enable pins
ISR (PORTA_PORT_vect)
{
	SPINDLE_ENABLE_SWITCH_PORT.INTFLAGS = PORT_INT_gm; //clear interrupt
	//read pins and update flags
	dir_spindle = ((SPINDLE_DIRECTION_SWITCH_PORT.IN & SPINDLE_DIRECTION_SWITCH_MASK) >> SPINDLE_DIRECTION_SWITCH_PIN_POSITION);
	run_spindle = ((SPINDLE_ENABLE_SWITCH_PORT.IN & SPINDLE_ENABLE_SWITCH_MASK) >> SPINDLE_ENABLE_SWITCH_PIN_POSITION) | dir_spindle;			
	run_feed = ((FEED_DIRECTION_SWITCH_PORT.IN & FEED_DIRECTION_SWITCH_MASK) >> FEED_DIRECTION_SWITCH_PIN_POSITION);
}

//setup interrupt for changes on enable pins
ISR (PORTB_PORT_vect)
{
	FEED_ENABLE_SWITCH_PORT.INTFLAGS = PORT_INT_gm; //clear interrupt
	run_feed = ((FEED_ENABLE_SWITCH_PORT.IN & FEED_ENABLE_SWITCH_MASK) >> FEED_ENABLE_SWITCH_PIN_POSITION); //read pins
}	



///////////////////////////////////////////////////////////////////////////////
///////////////////////// START of MAIN ///////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
int main(void)
{

	//set outputs
	PORTMUX.CTRLC = PORTMUX_TCA03_ALTERNATE_gc; //TCA03 alt pin
	STEP_SPINDLE_PORT.OUTCLR = STEP_SPINDLE_MASK;
	STEP_SPINDLE_PORT.DIRSET = STEP_SPINDLE_MASK;
	
	PORTMUX.CTRLD = PORTMUX_TCB0_ALTERNATE_gc; //TCB0 alt pin
	STEP_FEED_PORT.OUTCLR = STEP_FEED_MASK;
	STEP_FEED_PORT.DIRSET = STEP_FEED_MASK;
	
	SPINDLE_ENABLE_CONTROL = PORT_INVEN_bm;
	SPINDLE_ENABLE_PORT.OUTCLR = SPINDLE_ENABLE_MASK;
	SPINDLE_ENABLE_PORT.DIRSET = SPINDLE_ENABLE_MASK;

	FEED_ENABLE_CONTROL = PORT_INVEN_bm;
	FEED_ENABLE_PORT.OUTCLR = FEED_ENABLE_MASK;
	FEED_ENABLE_PORT.DIRSET = FEED_ENABLE_MASK;
	
	//set input, pull-ups on pins, interrupt from pins, etc
	SPINDLE_ENABLE_SWITCH_CONTROL =		PORT_ISC_BOTHEDGES_gc | PORT_PULLUPEN_bm |PORT_INVEN_bm;
	SPINDLE_DIRECTION_SWITCH_CONTROL =	PORT_ISC_BOTHEDGES_gc | PORT_PULLUPEN_bm | PORT_INVEN_bm;
	FEED_ENABLE_SWITCH_CONTROL =		PORT_ISC_BOTHEDGES_gc | PORT_PULLUPEN_bm | PORT_INVEN_bm;
	FEED_DIRECTION_SWITCH_CONTROL =		PORT_ISC_BOTHEDGES_gc | PORT_PULLUPEN_bm | PORT_INVEN_bm;


	//set CPU to run at full speed (default is 3.3MHz)
	cpu_clock_init();
	
	//initialise 
	init_ADC();
	led.init();
	bcd_input.init();
	
	
	//on startup if any switches (enable or direction are enabled, halt and wait for safe state
	if (interlock == 1)
	{
		while ( (SPINDLE_ENABLE_SWITCH_PORT.IN & SPINDLE_ENABLE_SWITCH_MASK) >= 1 
		|| (SPINDLE_DIRECTION_SWITCH_PORT.IN & SPINDLE_DIRECTION_SWITCH_MASK) >= 1
		|| (FEED_ENABLE_SWITCH_PORT.IN & FEED_ENABLE_SWITCH_MASK) >= 1
		|| (FEED_DIRECTION_SWITCH_PORT.IN & FEED_DIRECTION_SWITCH_MASK) >= 1 )
		
		{
				//wait
		}
		
		interlock = 0;
		
		//align the spindle encoder
		syncronise_encoder();
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
			spindle_clock_divider = BINS-1; //div_max
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
				feed_rate = bcd_input.bcd_to_int(); //read BCD counter - 1/1000ths of a millimeter
				analogue_value = (255 - get_adc(3)); //read potentiometer
				
				//set analogue value based on divider
				analogue_value = (analogue_value / STANDARD_CONTROL_STEPS) + 1;
				
				//calculate ideal RPM from analogue value
				if ((uint16_t)analogue_value < (((uint16_t)MAX_RPM - FINE_CONTROL_RPM) / 100))
				{
					spindle_speed = (uint16_t)((uint16_t)MAX_RPM - ((uint16_t)analogue_value * 100));
				} 
				else
				{
					spindle_speed =  (uint16_t)(FINE_CONTROL_RPM - ((uint16_t)analogue_value - ((((uint16_t)MAX_RPM - FINE_CONTROL_RPM) / 100) + 1)) * FINE_CONTROL_STEPS);
				}
				
				//calculate closest value clock settings to achieve requested / ideal rpm
				uint8_t temp_integer = 0;
				uint16_t temp_value = -1; //max value
						
				//calculate for each clock divider and keep the one that's closest to the desired rpm
				for (uint8_t n = 0; n < BINS; n++)
				{
					//for each clock divider calculate the integer that gives rpm closest to requested
					uint8_t temp_div_integer = (uint8_t)((float)CPU_SPEED / ((float)spindle_clock_divider_array[n] * (float)spindle_speed * (float)SPINDLE_STEPS));
					//calculate the rpm values to allow calculation of the closest value
					int16_t temp_div_rpm = (int16_t)(((float)CPU_SPEED / ((float)spindle_clock_divider_array[n]*float(temp_div_integer)*(float)SPINDLE_STEPS)));
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
				
				ramp_loops = (uint16_t)(RAMP_ACCELERATION * ((float)CPU_SPEED / ((float)spindle_clock_divider_array[spindle_clock_divider] * analogue_value * MAX_RPM)));
				
				//update the clock divider.
				cli();
				TCA0.SPLIT.CTRLA = 0;
				TCA0.SPLIT.CTRLA |= divider_array[spindle_clock_divider];
				sei();
				
				
				led.send_number(rpm_out(analogue_value)); //sends the rpm
			}
			
		}
		
		//////////////////////////////////////////////////////////////////////////
		//spindle only, allow reverse only when slow running
		while ( (interlock == 0 && run_spindle == 1 && run_feed == 0 && dir_spindle == 0) 
			 || (interlock == 0 && run_spindle == 1 && run_feed == 0 && dir_spindle == 1 && spindle_speed <= 20) )
		{
			//_delay_ms(MIN_SPINDLE_DRIVER_PULSE_DELAY);
			//syncronise_encoder();
			
			//check for previous running
			if (spindle_running == 0)
			{				
				//enable spindle motor
				SPINDLE_ENABLE_PORT.OUTSET = SPINDLE_ENABLE_MASK;
				spindle_running = 1;
				_delay_us(MIN_SPINDLE_DRIVER_PULSE_DELAY); //delay to allow init of driver

				//enable timers
				TCA0.SPLIT.CTRLA |= TCB_ENABLE_bm;
				
				//ramp up
				ramp_spindle_up();	
			}
					
			while (interlock == 0 && run_spindle == 1 && spindle_ramp_counter == 0 && run_feed == 0)
			{
				//hold here until interrupt
			}
			
			//if spindle off after interrupt, then ramp down the spindle
			if (run_spindle == 0)
			{
				//ramp speed down
				ramp_spindle_down();
				
				//disable spindle
				SPINDLE_ENABLE_PORT.OUTCLR = SPINDLE_ENABLE_MASK;
				spindle_running = 0;
			}
			
			
		}
		
		//////////////////////////////////////////////////////////////////////////
		//feed only
		while (interlock == 0 && run_spindle == 0 && run_feed == 1)
		{
			_delay_ms(MIN_SPINDLE_DRIVER_PULSE_DELAY);
			
			//set feed rate
			TCB0.CCMP = FEED_DEFAULT_PER; //set pulse length
			
			//enable feed motor
			FEED_ENABLE_PORT.OUTSET = FEED_ENABLE_MASK;
			_delay_us(MIN_SPINDLE_DRIVER_PULSE_DELAY); //delay for allow driver startup

			//enable timers
			TCB0.CTRLA |= TCB_ENABLE_bm;
			
			while (interlock == 0 && run_spindle == 0 && run_feed == 1)
			{
				//hold here until interrupt
			}
			
			//disable feed motor
			FEED_ENABLE_PORT.OUTCLR = FEED_ENABLE_MASK;
		}
		
		//////////////////////////////////////////////////////////////////////////
		//standard spindle with feed - feed rate number is arbitrary value
		//spindle must be started / running before entering this mode
		while (interlock == 0 && run_spindle == 1 && spindle_ramp_counter == 0 && run_feed == 1 && spindle_speed > 20)
		{
			//set feed rate
			TCB0.CCMP = FEED_DEFAULT_PER*20; //set clock turn over slower speed during cuts
			
			//enable feed timer
			TCB0.CTRLA |= TCB_ENABLE_bm; //enable clock	
			
			//enable feed motor
			FEED_ENABLE_PORT.OUTSET = FEED_ENABLE_MASK;
			_delay_us(MIN_SPINDLE_DRIVER_PULSE_DELAY); //delay for allow driver startup
	
			
			while (interlock == 0 && run_spindle == 1 && spindle_ramp_counter == 0 && run_feed == 1 && spindle_speed > 20)
			{
				//hold here until interrupt
			}
			
			//disable feed motor
			FEED_ENABLE_PORT.OUTCLR = FEED_ENABLE_MASK;
			
			//if spindle off after interrupt, then ramp down the spindle
			if (run_spindle == 0)
			{
				ramp_spindle_down();
				
				//disable spindle & feed
				SPINDLE_ENABLE_PORT.OUTCLR = SPINDLE_ENABLE_MASK;
				FEED_ENABLE_PORT.OUTCLR = FEED_ENABLE_MASK;
				spindle_running = 0;
			}
		}
		
		//////////////////////////////////////////////////////////////////////////
		//Engaged spindle with feed / slow spindle
		while (interlock == 0 && run_spindle == 1 && run_feed == 1 && spindle_speed <= 20)
		{
			_delay_ms(MIN_SPINDLE_DRIVER_PULSE_DELAY);
			
			//disable timers
			TCA0.SPLIT.CTRLA &= ~TCA_SPLIT_ENABLE_bm;
			TCB0.CTRLA &= ~TCB_ENABLE_bm;	
			TCA0.SPLIT.CTRLB = 0; //disable timer interrupt		
			
			//calculate stepping ratio
			//convert feed_rate to the number of steps the spindle makes per step the feed makes
			float spindle_steps_per_feed_step = (float)SPINDLE_STEPS / ((float)feed_rate / FEED_BCD_DIVISOR / FEED_MM_PER_STEP);
			
			volatile float current_decimal = spindle_steps_per_feed_step;
			volatile uint16_t current_spindle_steps = (uint16_t)spindle_steps_per_feed_step; 
			
			//enable spindle & feed motor
			SPINDLE_ENABLE_PORT.OUTSET = SPINDLE_ENABLE_MASK;
			spindle_running = 1;
			FEED_ENABLE_PORT.OUTSET = FEED_ENABLE_MASK;
			_delay_us(MIN_SPINDLE_DRIVER_PULSE_DELAY); //delay for allow driver startup
			
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
					STEP_SPINDLE_PORT.OUTTGL = STEP_SPINDLE_MASK;  //Toggle step GPIO 
					_delay_us(MIN_SPINDLE_DRIVER_PULSE_DELAY); //delay
					
					if (run_spindle == 0)
					{
						break;
					}
				}
				
				STEP_FEED_PORT.OUTTGL = STEP_FEED_MASK;  //Toggle feed step 
				_delay_us(MIN_SPINDLE_DRIVER_PULSE_DELAY); //delay
			}
			
			//disable spindle & feed
			SPINDLE_ENABLE_PORT.OUTCLR = SPINDLE_ENABLE_MASK;
			spindle_running = 0;
			FEED_ENABLE_PORT.OUTCLR = FEED_ENABLE_MASK;
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
	
	uint16_t out_number = uint16_t(((float)CPU_SPEED / ((float)spindle_clock_divider_array[spindle_clock_divider]*float(per_size)*(float)SPINDLE_STEPS)));
	
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
			SPINDLE_ENABLE_PORT.OUTCLR = SPINDLE_ENABLE_MASK;
			spindle_running = 0;

			//exit ramp up
			return;
		}

	}
}


void ramp_spindle_down()
{
	//ramp the frequency
	for (uint8_t n = analogue_value; n <= 92; n++)
	{
		TCA0.SPLIT.HPER = n;
		TCA0.SPLIT.HCMP0 = n/2; //HPER divide by 2 ~50% duty cycle
		
		spindle_ramp_counter = ramp_loops;
		
		TCA0.SPLIT.INTCTRL |= TCA_SPLIT_HUNF_bm; //enable interrupt flag
		
		while (spindle_ramp_counter != 0)
		{
			//hold here until interrupt loop finished
		}

		TCA0.SPLIT.INTCTRL &= ~(TCA_SPLIT_HUNF_bm); //disable interrupt
	}
}


void syncronise_encoder()
{
	//position lock the spindle
	SPINDLE_ENABLE_PORT.OUTSET = SPINDLE_ENABLE_MASK;
	spindle_running = 1;
	
	//step the spindle to ensure that the encoder is aligned
	for (uint8_t i = 0; i <= 6; i++)
	{
		STEP_SPINDLE_PORT.OUTTGL = STEP_SPINDLE_MASK;
		_delay_ms(MIN_SPINDLE_DRIVER_PULSE_DELAY);
	}
	
	//disable the spindle
	STEP_SPINDLE_PORT.OUTCLR = STEP_SPINDLE_MASK;
	SPINDLE_ENABLE_PORT.OUTCLR = SPINDLE_ENABLE_MASK;
	spindle_running = 0;
}
