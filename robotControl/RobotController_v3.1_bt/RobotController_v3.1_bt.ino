/******************************************************************************************************************* 
 *   This program is the main control program for making a differential robot follow a wall. The robot has 5 range
 *   sensors (two on the left, two on the right and one at the front) to measure distances from the walls on each 
 *   side. This version performs the following tasks.
 *    
 *    1. Adds the calibration routine for the front sensor.
 *    2. Write function for measuring the distance from the wall.
 *    3. Write function for measuring the orientation (non-parallelism) of the robot.
 *    4. Remove the ASCII Messenger library and instead add code for taking queues from button presses.
 *    5. Write code for implementing PID controller for corridore following.
 *        - Two PID controllers will be used.
 *        - First will correct the orientation error
 *        - Second will correct the distance error
 *        
 *        Driver Side        Arduino Side    Robot Motors
 *            D0    --->  OC1A (PD3)  --->  Left Motor-A  
 *            D1    --->  OC1B (PB3)  --->  Left Motor-B
 *            D2    --->  OC2A (PB1)  --->  Right Motor-A
 *            D3    --->  OC2B (PB2)  --->  Right Motor-B
 *    
 *    Author:   Muhammad Taha
 *    Dated:    05/10/2023
 *    version:  3.1 (with Bluetooth control)
 *    ************************************************************************************************************/
#define LEFT  0
#define RIGHT 1

#define LEFT_MOTOR    0
#define RIGHT_MOTOR   1

#define RIGHT_BUTTON  8   // Button attached to D8 which is pulled up internally
#define LEFT_BUTTON   7   // Button attached to D7 which is pulled up internally

#define RED_LED     17    // Red LED connected at pin 17 (A3)
#define BLUE_LED    16    // Red LED connected at pin 16 (A2)

#define BUZZER  6   // PD6 for Buzzer

#include <PID_v1.h>
#define PI_CONTROL_ON     //choose at compile time whether to use PID or not    

/************************** Library for Messaging ********************************/
 // Include AsciiMassageParser.h and AsciiMassagePacker.h for ASCII format massage packing and parsing.
//#include <AsciiMassageParser.h>

// Instantiate an AsciiMassageParser and an AsciiMassagePacker.
//AsciiMassageParser inbound;

/*********************************************************************************/
#define DEBUG_PRINTS
/************************ PID Definitions and Variables **************************/
// defines variables
double robot_deviation;             // Robot deviation. Input to PID_dev
double PID_dev_output;             // Output of PID_dev
double reference_deviation = 0.0;    // Setpoint: desired distance (in cm) from the center of the corridore or the wall

double orientation;                 // Measured orientation from the wall
double reference_orientation = 0;   // Setpoint: This would be 0 for when the robot is parallel to the wall
double PID_ornt_output;             // Output of PID_ornt

int DesiredVRobot = 0;              // This is the desired linear velocity if the robot correctly positioned
int state = 0;

// The gains for the two PID controllers   

//double Kp_dist = 25.0, Ki_dist = 0.0, Kd_dist = 7.5;  (works marvelously without the orientation controller
//double Kp_ornt = 2.5, Ki_ornt = 0.0, Kd_ornt = 0.5;
double Kp_dist = 25.0, Ki_dist = 0.0, Kd_dist = 7.5;
double Kp_ornt = 2.5, Ki_ornt = 0.0, Kd_ornt = 0.75;

//bool left_button_pressed = false;
int SampleTime = 100;

PID PID_dev(&robot_deviation, &PID_dev_output, &reference_deviation, Kp_dist, Ki_dist, Kd_dist, DIRECT);
PID PID_ornt(&orientation, &PID_ornt_output, &reference_orientation, Kp_ornt, Ki_ornt, Kd_ornt, DIRECT);

void PID_initiallize()
{
  PID_dev.SetSampleTime(SampleTime);
  PID_dev.SetOutputLimits(-250.0,250.0);  // 25% of the robot's Motor velocities

  PID_ornt.SetSampleTime(SampleTime);
  PID_ornt.SetOutputLimits(-250.0,250.0);  // 25% of the robot's Motor velocities
}

void setup() 
{
  /************************* Step 1: Initiallize modules ************************************/
  #ifdef DEBUG_PRINTS
    Serial.begin(115200);
    Serial.println("Starting the robot");
  #endif
  
  init_range_sensors();
  initiallize_timers();
  stop_robot();
  //set_servo_angle(180);
  PID_initiallize();
  pinMode(RIGHT_BUTTON, INPUT_PULLUP); // This pin will read 0 if the button is pressed
  pinMode(LEFT_BUTTON, INPUT_PULLUP); // This pin will read 0 if the button is pressed
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT); 
  loadCalibration();
  Serial.println("Caliration loaded!");
  /******************************************************************************************/

      /*Serial.println("Testing robot motion!");
      
      Serial.println("Right Motor Reverse.");
      set_motor_speed(RIGHT_MOTOR, 800);     
      delay(3000);
      Serial.println("Stop Motors.");
      stop_motors();
      delay(1000);
      Serial.println("Right Motor Reverse.");
      set_motor_speed(RIGHT_MOTOR, -800);
      delay(3000);
      Serial.println("Motors Stop.");
      stop_motors();
      delay(1000);
      
      Serial.println("Left Motor Forward.");
      set_motor_speed(LEFT_MOTOR, 800);
      delay(3000);
      Serial.println("Motors Stop.");
      stop_motors();
      delay(1000);
      Serial.println("Left Motor Reverse.");
      set_motor_speed(LEFT_MOTOR, -800);
      delay(3000);
      Serial.println("Motors Stop.");
      stop_motors();
      delay(1000);*/

     

}

 /**************************** End of program ******************************/

void loop() 
{
  /**************** Parse incoming messages and set appropriate flags **************************/
  if(Serial.available())
  {
    String temp = Serial.readStringUntil('\n');
     // Receive command "GetRanges"
    if(temp.equals("GetRanges"))
    {
      read_range_sensors();
      print_range_sensors();      
    }

    // Receive command "MakeSound"
    if(temp.equals("MakeSound"))
    {
      beep_buzzer();
      blink_led(BLUE_LED, 5);
      //tone(BUZZER, 2000, 250);
    }

    // Receive command "TestServo"
    if(temp.equals("TestServo"))
    {
      Serial.println("Enabling Servo!");
      enable_servo();
      Serial.println("Setting Angle 0.");
      set_servo_angle(0);
      delay(1000);
      Serial.println("Setting Angle 90.");
      set_servo_angle(90);
      delay(1000);
      Serial.println("Setting Angle 180.");
      set_servo_angle(180);
      delay(1000);
      Serial.println("Setting Angle 0.");
      set_servo_angle(0);
      delay(1000);
      Serial.println("Disabling Servo!");
      disable_servo();
    }    
    
    // Receive command "LoadCalibration"
    if ((temp.equals("LoadCalibration"))  && (state == 0))  /// If we want to load previously saved config
    {
      state = 1;
      loadCalibration();
      Serial.println("Caliration loaded!");
    }

    // Receive command "TestMotors"
    if(temp.equals("TestMotors"))
    {
      Serial.println("Testing robot motion!");
      
      Serial.println("Right Motor Forward.");
      set_motor_speed(RIGHT_MOTOR, 800);     
      delay(3000);
      Serial.println("Stop Motors.");
      stop_motors();
      delay(1000);
      Serial.println("Right Motor Reverse.");
      set_motor_speed(RIGHT_MOTOR, -800);
      delay(3000);
      Serial.println("Motors Stop.");
      stop_motors();
      delay(1000);
      
      Serial.println("Left Motor Forward.");
      set_motor_speed(LEFT_MOTOR, 800);
      delay(3000);
      Serial.println("Motors Stop.");
      stop_motors();
      delay(1000);
      Serial.println("Left Motor Reverse.");
      set_motor_speed(LEFT_MOTOR, -800);
      delay(3000);
      Serial.println("Motors Stop.");
      stop_motors();
      delay(1000);
      
    }
    
    // This command will move the robot forward, then backwards, rotate left and then rotate right before stopping
    if(temp.equals("TestRobotMotion"))
    {
      Serial.println("Testing robot motion!");
      Serial.println("Moving forward and right.");
      ramp_robot_speed(800);
      delay(1000);
      ramp_robot_speed(0);
      stop_robot();
      delay(1000);
      ramp_robot_speed(-800);
      delay(1000);
      ramp_robot_speed(0);
      stop_robot();
      delay(1000);
      /*ramp_robot_omega(400);
      delay(2000);
      ramp_robot_omega(0);
      stop_robot();
      delay(1000);
      ramp_robot_omega(-400);
      delay(2000);
      ramp_robot_omega(0);
      stop_robot();*/
      /*Serial.println("Moving forward and left.");
      delay(5000);
      ramp_robot_speed(600);
      ramp_robot_omega(-50);
      delay(2000);
      ramp_robot_speed(0);
      ramp_robot_omega(0);
      stop_robot();
      */
      /*Serial.println("Rotating right.");
      delay(1000);
      ramp_robot_omega(400);
      delay(2000);
      ramp_robot_omega(0);
      //stop_robot();
      Serial.println("Rotating left.");
      delay(2000);
      ramp_robot_omega(-400);
      delay(2000);
      ramp_robot_omega(0);
      stop_robot();*/
    }

    if((temp.equals("Abort")))
    {
      state = 0;
      PID_ornt.SetMode(MANUAL);
      DesiredVRobot = 0;
      ramp_robot_speed(0);
      ramp_robot_omega(0);
      stop_robot();
      Serial.println("Aborting.");
    }

    if((temp.equals("FollowWall")) && (state == 1))
    {
      follow_wall_duration(10000, RIGHT, 8.0, 600);  // follow the wall for 10 seconds
    }

    if((temp.equals("Pivot"))  && (state == 1))  /// Testing the pivoting code
    {
      pivot_robot_duration(800, 200);
      Serial.println("Pivot complete.");
    }
    if((temp.equals("TurnRobot"))  && (state == 1))  /// Testing the turning code
    {
      turn_robot(500, 600, 200);
      Serial.println("Turn complete.");
    }
    if((temp.equals("TestMission"))  && (state == 1))  /// Testing the test mission code
    {
      /** Step 1: Move in the first leg of the corridore in the Map **/
      follow_wall_till_end(RIGHT,10.0,100,600); /// Move with speed 600 keeping a distance of 10.0 cm from the RIGHT wall. 
                                                /// Stop 10cm (100 mm) before the end of the corridore.
      /* Make a U-turn to the left after first segment */  
      turn_robot(300, 230, -200);         /// Turn with 230 Linear speed and 200 angular speed (Anti-Clockwise) 
      //pivot_robot_duration(650,-200);
      
    }
  }

  /*if(button_is_pressed(LEFT_BUTTON))  /// If left button is pressed
    left_button_pressed = true;*/

  if(button_is_pressed(LEFT_BUTTON))
  {
    //left_button_pressed = false;
    loadCalibration();
    Serial.println("Caliration loaded!");
    delay(3000);
    /** Step 1: Move in the first leg of the corridore in the Map **/
    follow_wall_till_end(RIGHT,10.0,300,600); /// Move with speed 600 keeping a distance of 10.0 cm from the RIGHT wall. 
                                                /// Stop 30cm (300 mm) before the end of the corridore.
    delay(100);
    /* Make a U-turn to the left after first segment */  
    //turn_robot(1000, 230, -300);         /// Turn with 230 Linear speed and 200 angular speed (Anti-Clockwise) 
    pivot_robot_duration(380,-300);

    /** Step 2: Move in the first leg of the corridore in the Map **/
    follow_wall_till_end(RIGHT,10.0,250,600); /// Move with speed 600 keeping a distance of 10.0 cm from the RIGHT wall.                                                 /// Stop 30cm (300 mm) before the end of the corridore.
                                              /// Stop 25cm (250 mm) before the end of the corridore.
    delay(100);
    /* Make a U-turn to the left after the 2nd segment */  
    //turn_robot(1000, 230, -300);         /// Turn with 230 Linear speed and 200 angular speed (Anti-Clockwise) 
    pivot_robot_duration(380,-300);
    delay(100);

    /* Move forward without PID */
    follow_wall_duration(1000, RIGHT, 10.0, 600);
    ramp_robot_speed(0);
    stop_robot();
    delay(100);
  }
}

void follow_wall_duration(unsigned long m_sec, unsigned int dir, double dist, int desired_velocity)
{
  
  reference_deviation = dist;        // Required distance from the wall
  reference_orientation = 0.0;      // Required orientation
  DesiredVRobot = desired_velocity;
  PID_dev.SetMode(AUTOMATIC);
  PID_ornt.SetMode(AUTOMATIC);
  unsigned long t_st = millis();
  while((millis() - t_st) < m_sec)
  {
    if((millis() % 100) == 0)
    {
      read_range_sensors();  // Read Sensors
      if(dir == RIGHT)
      {
        robot_deviation = get_avg_distance(RIGHT);
        orientation = (double) get_orientation(RIGHT);
      }
      else if(dir == LEFT)
      {
        robot_deviation = get_avg_distance(LEFT);
        orientation = (double) get_orientation(LEFT);
      }
      
      /* Commenting out this older block of code and replacing with the new one */
      PID_ornt.Compute();
      PID_dev.Compute();
      //set_robot_speed(DesiredVRobot - (int) (abs(PID_ornt_output) * 0.50) );
      set_robot_speed(DesiredVRobot - (int) (abs(PID_dev_output) * 0.25) );
      //set_robot_speed(DesiredVRobot);
      //int new_omega = (int) (0.3 * PID_ornt_output) + (0.7 * PID_dev_output);
      int new_omega = (int) (PID_dev_output);
      //int new_omega = (int) (PID_ornt_output);
      set_robot_omega(-1 * new_omega);
      /**************************************************************************/
      /************************* This is the new block ************************  
      PID_ornt.Compute();
      PID_dev.Compute();
      set_robot_speed(DesiredVRobot - (int) (abs(PID_ornt_output) * 0.50) );
      
      if(abs(orientation) > 20.0) // if error in orientation is more than 20.0 then fix orientation only
      {
        int new_omega = (int) PID_ornt_output;
        set_robot_omega(-1 * new_omega);
        Serial.print("Fix orient ");
      }
      else
      {
        int new_omega = (int) PID_dev_output;
        set_robot_omega(-1 * new_omega);
        Serial.print("Fix dist ");
      }
      /************************************************************************/
      Serial.print(" Orn Err: ");
      Serial.print(orientation);
      Serial.print(" Dist: ");
      Serial.print(robot_deviation);
      Serial.print(" Vrob: ");
      Serial.print(get_robot_speed());
      Serial.print(" Omega: ");
      Serial.println(get_robot_omega());
    }
    /*if((millis() % 245) == 0)
    {
      Serial.print(" Orn Err: ");
      Serial.print(orientation);
      Serial.print(" Dist: ");
      Serial.print(robot_deviation);
      Serial.print(" Vrob: ");
      Serial.print(get_robot_speed());
      Serial.print(" Omega: ");
      Serial.println(get_robot_omega());
    }  */

  }
  
  ramp_robot_speed(0);
  ramp_robot_omega(0);
  stop_robot();
  Serial.println("Stopping Robot!!");  
  PID_dev.SetMode(MANUAL);
  PID_ornt.SetMode(MANUAL);
}

/*   This function makes the robot follow a wall on the side defined by 'dir' at a distance 'dist'. The robot 
 *   follows the wall untill it locates a turning on the opposite side.
 */
void follow_wall_till_turn(unsigned int dir, double dist, int desired_velocity)
{
  reference_deviation = dist;        // Required distance from the wall
  reference_orientation = 0.0;      // Required orientation
  DesiredVRobot = desired_velocity;
  PID_dev.SetMode(AUTOMATIC);
  PID_ornt.SetMode(AUTOMATIC);
  
  int turn_dist;  /// Distance measured by the sensors looking for the turn
  read_range_sensors();  // Read Sensors
  if(dir == RIGHT)
    turn_dist = get_avg_distance(LEFT);
  else
    turn_dist = get_avg_distance(RIGHT);
  
  while(turn_dist < 50) /// It is not good to use a fixed value here but the assumption is that on a turn this distance will be definitely larger than 50 cm
  {
    if((millis() % 100) == 0)
    {
      read_range_sensors();  // Read Sensors
      if(dir == RIGHT)
      {
        turn_dist = get_avg_distance(LEFT);
        robot_deviation = get_avg_distance(RIGHT);
        orientation = get_orientation(RIGHT);
      }
      else if(dir == LEFT)
      {
        turn_dist = get_avg_distance(RIGHT);
        robot_deviation = get_avg_distance(LEFT);
        orientation = get_orientation(LEFT);
      }
      
    
      PID_ornt.Compute();
      PID_dev.Compute();
      set_robot_speed(DesiredVRobot - (int) (abs(PID_ornt_output) * 0.50) );
      int new_omega = (int) (0.7 * PID_ornt_output) + (0.3 * PID_dev_output);
      set_robot_omega(-1 * (int) new_omega);
      Serial.print("Orn Err: ");
      Serial.print(orientation);
      Serial.print(" Dist: ");
      Serial.print(robot_deviation);
      Serial.print(" Vrob: ");
      Serial.print(get_robot_speed());
      Serial.print(" Omega: ");
      Serial.println(get_robot_omega());  
    }

  }
  
  ramp_robot_speed(0);
  ramp_robot_omega(0);
  stop_robot();
  PID_dev.SetMode(MANUAL);
  PID_ornt.SetMode(MANUAL);

}
/*  This function makes the robot follow a corridore till it reaches it's end. The distance reading
 *  at the front sensor that signifies the end of the corridore is supplied as an input argument.
 *  Other inputs are:
 *      - deviation -> deviation from the center of the corridore
 *      - end_dist  -> distance at the front that signifies the end of the corridore
 *      - desired_velocity -> Desired linear speed of the robot
 * 
 */
void follow_corridore_till_end(double deviation, double end_dist, int desired_velocity)
{
  reference_deviation = deviation;   // Required distance form the center of the corridore
  reference_orientation = 0.0;      // Required orientation
  DesiredVRobot = desired_velocity;
  PID_dev.SetMode(AUTOMATIC);
  PID_ornt.SetMode(AUTOMATIC);
  read_range_sensors();  // Read Sensors
  do
  {
    if((millis() % 100) == 0)
    {
      read_range_sensors();  // Read Sensors
      robot_deviation = get_dist_deviation();
      orientation = get_orientation(RIGHT);
    
 
      PID_ornt.Compute();
      PID_dev.Compute();
      set_robot_speed(DesiredVRobot - (int) (abs(PID_ornt_output) * 0.50) );
      int new_omega = (int) (0.7 * PID_ornt_output) + (0.3 * PID_dev_output);
      set_robot_omega(-1 * (int) new_omega);
    }

  }while(dist_at_front() > end_dist);
  
  ramp_robot_speed(0);
  ramp_robot_omega(0);
  stop_robot();
  PID_dev.SetMode(MANUAL);
  PID_ornt.SetMode(MANUAL);  
  
}

/*  This function makes the robot follow a wall till it reaches the end of the corridore. The distance reading
 *  at the front sensor that signifies the end of the corridore is supplied as an input argument.
 *  Other inputs are:
 *      - dir -> RIGHT or LEFT sides
 *      - deviation -> deviation from the desired distance from the wall
 *      - end_dist  -> distance at the front that signifies the end of the corridore
 *      - desired_velocity -> Desired linear speed of the robot
 * 
 */
void follow_wall_till_end(unsigned int dir, double deviation, double end_dist, int desired_velocity)
{
  reference_deviation = deviation;   // Required distance form the center of the corridore
  reference_orientation = 0.0;      // Required orientation
  DesiredVRobot = desired_velocity;
  PID_dev.SetMode(AUTOMATIC);
  PID_ornt.SetMode(AUTOMATIC);
  read_range_sensors();  // Read Sensors
  do
  {
    if((millis() % 100) == 0)
    {
      read_range_sensors();  // Read Sensors
      if(dir == RIGHT)
      {
        robot_deviation = get_avg_distance(RIGHT);
        orientation = get_orientation(RIGHT);
      }
      else
      {
        robot_deviation = get_avg_distance(LEFT);
        orientation = get_orientation(LEFT);
      }
      PID_ornt.Compute();
      PID_dev.Compute();
      set_robot_speed(DesiredVRobot - (int) (abs(PID_ornt_output) * 0.50) );
      int new_omega = (int) (0.7 * PID_ornt_output) + (0.3 * PID_dev_output);
      set_robot_omega(-1 * (int) new_omega);
      Serial.print(" Orn Err: ");
      Serial.print(orientation);
      Serial.print(" Dist: ");
      Serial.print(robot_deviation);
      Serial.print(" Vrob: ");
      Serial.print(get_robot_speed());
      Serial.print(" Omega: ");
      Serial.print(get_robot_omega());  
      Serial.print(" Front: ");
      Serial.println(dist_at_front());  
    }

  }while(dist_at_front() > end_dist);
  
  ramp_robot_speed(0);
  ramp_robot_omega(0);
  stop_robot();
  PID_dev.SetMode(MANUAL);
  PID_ornt.SetMode(MANUAL);  
}


/*  This function pivots the robot clockwise or anticlockwise (based on sign of 'omg'.
 *  The magnitude of 'omg' determines the speed of rotation. 
 */
void pivot_robot_duration(unsigned long m_sec, int omg)
{
   ramp_robot_omega(omg);
   unsigned long t_st = millis();
   while((millis() - t_st) < m_sec);  // wait till duration complete
   
   ramp_robot_omega(0);
   stop_robot();
}
/** This function will make the robot take a wide turn. The width of the turning will 
 *  depend on the combination of linear and the angular speeds. 
 */
void turn_robot(unsigned long m_sec, int lin_spd, int ang_spd)
{
  ramp_robot_speed(lin_spd);
  ramp_robot_omega(ang_spd);
  unsigned long t_st = millis();
  while((millis() - t_st) < m_sec);  // wait till duration complete
  
  ramp_robot_omega(0);
  ramp_robot_speed(0);
  
  stop_robot();
}
