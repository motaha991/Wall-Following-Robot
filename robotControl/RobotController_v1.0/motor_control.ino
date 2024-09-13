#define LEFT_MOTOR    0
#define RIGHT_MOTOR   1
#define FORWARD   1
#define BACKWARDS 0
/*  This function sets the speed of the two motors. It takes two arguments, 
 *    1. Motor No. (LEFT_MOTOR 0 or RIGHT_MOTOR 1) 
 *    2. Motor Speed (0 +/- 1023)  
 */
void set_motor_speed(unsigned char motor, int motor_speed)
{
  if(motor == RIGHT_MOTOR)
  { 
    if(motor_speed < 0) // For reverse motion
    {
      set_right_motor_speed(abs(motor_speed), BACKWARDS);
    }
    else
    {
      set_right_motor_speed(abs(motor_speed), FORWARD);
    }
  
  }  
  else  /// for left motor
  {
    
    if(motor_speed < 0)
    {
      set_left_motor_speed(abs(motor_speed), BACKWARDS);
    }
    else
    {
      set_left_motor_speed(abs(motor_speed), FORWARD);      
    }
  }  
  
}


/*	This function takes as input the speed (0 to 1023) and direction of 
	the left motor as input and sets the corresponding registers to set 
	motor speed. Timer 1 controls the PWM for the left motor.
*/
void set_left_motor_speed(unsigned int sp, unsigned char dir)
{
	start_timer1();
	int duty_cycle = 50;
	if(dir == 1)	// Forward motion of the wheel
	{
    /*  Use the following settings for moving on the carpet. */
    /*int spd = (int) map_float(sp, 0, 1023, 0, 935);
		if(spd < 910)
		  duty_cycle = 50 + ((50 * spd) >> 10);
		else
			duty_cycle = 92;	// Maximum allowed Duty Cycle for the motor*/
     /********************************************************/
     
     /*  Use the following settings for moving on the Floor. */
    //int spd = (int) map_float(sp, 0, 1023, 0, 990);
    if(sp < 990)
      duty_cycle = 50 + ((50 * sp) >> 10);
    else
      duty_cycle = 97;  // Maximum allowed Duty Cycle for the motor
    /********************************************************/
	}
	else
	{
		if(sp < 990)
			duty_cycle = 50 - ((50 * sp) >> 10);
		else
			duty_cycle = 3;		// Minimum allowed Duty Cycle for the motor
	}
	
	set_timer1_dc(duty_cycle);
}
/*	This function takes as input the speed (0 to 1023) and direction of 
	the right motor as input and sets the corresponding registers to set 
	motor speed. Timer 2 controls the PWM for the right motor.
*/
void set_right_motor_speed(unsigned int sp, unsigned char dir)
{
	start_timer2();
	int duty_cycle = 50;
	if(dir == 1)	// Forward motion of the wheel
	{
     /*  Use the following settings for moving on the carpet. */
    int spd = (int) map_float(sp, 0, 1023, 0, 1000);
    if(spd < 980)
      duty_cycle = 50 + ((50 * spd) >> 10);
    else
      duty_cycle = 96;  // Maximum allowed Duty Cycle for the motor*/
     /********************************************************/
     
		/*if(sp < 990)
		duty_cycle = 50 + ((50 * sp) >> 10);
		else
		duty_cycle = 97;	// Maximum allowed Duty Cycle for the motor*/
	}
	else
	{
		if(sp < 990)
		duty_cycle = 50 - ((50 * sp) >> 10);
		else
		duty_cycle = 3;		// Minimum allowed Duty Cycle for the motor
	}
	
	set_timer2_dc(duty_cycle);
	
}

void stop_motors(void)
{
  stop_timer1();
  stop_timer2();
}

/*void get_left_motor_speed(unsigned int * speed, unsigned char * dir)
{
	unsigned char dir_prev = 0;
	unsigned int speed_prev = 0;
	unsigned char duty_cycle = get_timer1_dc();
	if(duty_cycle > 50)
	{
		dir_prev = 1;	// Motor was moving forward
		speed_prev = (duty_cycle-50)*20;
	}
		
}*/

/*  A mapping function which maps a value from one range to another range.
 *  Same as the Arduino 'map' function, but supporting floating point 
 *  numbers.
 */
float map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
