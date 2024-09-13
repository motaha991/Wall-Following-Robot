void initiallize_timers(void)
{
  init_timer1();
  init_timer2();
}
/*	This function configures Timer 1 to generate PWM Signal on both Channels A and B
	Fclk = 16 MHz
	Foc  ~= 4 kHz
	N = 8
	Mode = Phase Correct PWM
	Top  = 0xFF
	
*/
void init_timer1(void)
{
	DDRB |= (1<<PB1);		// Configure OC2A for output
	DDRB |= (1<<PB2);		// Configure OC2B for output
	TCCR1A = (1<<COM1A1)|(1<<COM1B1)|(1<<COM1B0)|(1<<WGM10);	// Phase Correct PWM, Non-inverting on Channel A, Inverting on Channel B
	//TCCR1A = (1<<COM1A1)|(1<<COM1B1)|(1<<COM1B0)|(1<<WGM10);	// Fast PWM, Non-inverting on Channel A, Inverting on Channel B
	TCCR1B = (1<<CS11);		// N=8
	//TCCR1B = (1<<WGM12)|(1<<CS11)|(1<<CS10);		// N=64
	//TCCR1B = (1<<CS12);		// N=256
	OCR1A = 127;			// initially the PWM duty cycle is 50%
	OCR1B = 127;
}

/* Initialize Timer2 for Phase Correct PWM on two channels.*/

void init_timer2(void)
{	
	DDRB |= (1<<PB3);		// Configure OC2A for output
	DDRD |= (1<<PD3);		// Configure OC2B for output
	//SPCR = 0;				// Disable SPI
	PRR &= ~(1<<PRTIM2);	// Enable Timer/Counter 2
	TCCR2A = (1<<COM2A1)|(1<<COM2B1)|(1<<COM2B0)|(1<<WGM20);	// Phase Correct PWM, Non-inverting on Channel A, Inverting on Channel B
	//TCCR2A = (1<<COM2A1)|(1<<COM2B1)|(1<<COM2B0)|(1<<WGM21)|(1<<WGM20);	// Fast PWM, Non-inverting on Channel A, Inverting on Channel B
	TCCR2B = (1<<CS21);		// N=8
	//TCCR2B = (1<<CS21)|(1<<CS20);	// N = 32 Foc0 = 980 Hz
	//TCCR2B = (1<<CS22);		// N=64
	OCR2A = 127;			// initially the PWM duty cycle is 50%
	OCR2B = 127;
}

void set_timer1_dc(unsigned int duty_cycle)	// duty_cycle for the non_inverting output
{
	unsigned int temp = (duty_cycle*255) / 100; 
	
	OCR1A = temp;	// duty_cycle ranges from 0 to 100
	OCR1B = temp;
}

void set_timer2_dc(unsigned char duty_cycle)
{
	unsigned char temp = (duty_cycle*255) / 100;
	
	OCR2A = temp;	// duty_cycle ranges from 0 to 100
	OCR2B = temp;
}

unsigned char get_timer1_dc(void)
{
	unsigned char dc = (OCR1A*100)/255;
	return(dc); 
}
unsigned char get_timer2_dc(void)
{
	unsigned char dc = (OCR2A*100)/255;
	return(dc);
}

void stop_timer1(void)
{
  TCCR1B = 0;    // No clock
  TCCR1A &= ~((1<<COM1A1)|(1<<COM1B1)|(1<<COM1B0));  // Clear these bits to disconnect COM1A and COM1B
  PORTB |= (1<<PB1);  // Set these pins so that the motor stops
  PORTB |= (1<<PB2);
}
void start_timer1(void)
{
  TCCR1B = (1<<CS11);    // N=8
  TCCR1A |= ((1<<COM1A1)|(1<<COM1B1)|(1<<COM1B0));  // Set these bits to re-connect COM1A and COM1B
}

void stop_timer2(void)
{
  TCCR2B = 0;    // No clock
  TCCR2A &= ~((1<<COM2A1)|(1<<COM2B1)|(1<<COM2B0)); // clear these bits to disconnect OC2A and OC2B 
  PORTB |= (1<<PB3); // Set these pins so that the motor stops
  PORTD |= (1<<PD3);
}
void start_timer2(void)
{
  TCCR2B = (1<<CS21);    // N=8
  TCCR2A |= ((1<<COM2A1)|(1<<COM2B1)|(1<<COM2B0)); // set these bits to re-connect OC2A and OC2B
}
