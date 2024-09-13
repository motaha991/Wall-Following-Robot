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
 *    Author:   Muhammad Taha
 *    Dated:    06/08/2023
 *    version:  3.1
 *    ************************************************************************************************************/
#define LEFT  0
#define RIGHT 1

#define BUTTON  2   // Button attached to D2 which is pulled up internally
#define LED_PIN 13  // LED is connected at D13 of Arduino Nano
#define TURN_ON_LED   digitalWrite(LED_PIN, HIGH)
#define TURN_OFF_LED  digitalWrite(LED_PIN, LOW)

#include <PID_v1.h>
#define PI_CONTROL_ON     //choose at compile time whether to use PID or not    


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

double Kp_dist = 2.5, Ki_dist = 0.05, Kd_dist = 0.0;
double Kp_ornt = 2.5, Ki_ornt = 0.0, Kd_ornt = 0.05;


int SampleTime = 100;

PID PID_dev(&robot_deviation, &PID_dev_output, &reference_deviation, Kp_dist, Ki_dist, Kd_dist, DIRECT);
PID PID_ornt(&orientation, &PID_ornt_output, &reference_orientation, Kp_ornt, Ki_ornt, Kd_ornt, DIRECT);

void PID_initiallize()
{
  PID_dev.SetSampleTime(SampleTime);
  PID_dev.SetOutputLimits(-150.0,150.0);  // 15% of the robot's Motor velocities

  PID_ornt.SetSampleTime(SampleTime);
  PID_ornt.SetOutputLimits(-250.0,250.0);  // 25% of the robot's Motor velocities
}

void setup() 
{
  /************************* Step 1: Initiallize modules ************************************/
  Serial.begin(115200);
  init_range_sensors();
  Serial.println("Starting the robot");
  initiallize_timers();
  stop_robot();
  PID_initiallize();
  pinMode(BUTTON, INPUT_PULLUP); // This pin will read 0 if the button is pressed
  pinMode(LED_PIN, OUTPUT); 
  /******************************************************************************************/

  /************************ Step 2: Calibrate Sensors or load calibration *******************
   *  If the robot is powered on with the button released, it will load the calibration offsets
   *  from the memory. If you need to re-calibrate the sensors, follow the listed steps below
   *  for calibration.
   *  
   *  01. Place the robot's right side parallel to and touching the wall.
   *  02. Power-on the robot with button pressed for new calibration.
   *  03. LED will turn ON to indicate right-side calibration succeeded.
   *  04. Now place the robot's left-side touching the wall and release the button
   *  05. The LED will turn OFF and robot will start to calibrate left-side sensors.
   *  06. Once the left-side sensors are calibrated, the LED will turn ON.
   *  07. Place the robot's front facing the wall. Press the button again.
   *  08. LED will turn OFF and robot will start the calibration of the front sensor.
   *  09. LED will finally turn ON to indicate that the calibration was successful.
   *  10. Release the button to finish the calibration. LED will trun OFF.
   */
  if(button_is_pressed(BUTTON)) // We need to calibrate the sensors anew.
  {
     int err_code = calibrate_range_sensors();
     if(err_code == 0)
     {
        Serial.println("Calibration successful!");
        TURN_ON_LED;  // Turn on the LED to indicate that calibration is successful
        state = 1;
        while(button_is_pressed(BUTTON));   // wait till the button is released
          TURN_OFF_LED;
     }
     else
     {
        Serial.print("Calibration failed! Error Code: ");        
        Serial.println(err_code);
        blink_led(10);  // blibk LED 10 times to indicate that there was an error
     }
  }
  else
  {
    loadCalibration();
    state = 1;
  }
  /******************************************************************************************/
  delay(5000);
  TURN_ON_LED;   // Now the robot is ready to move. Place the robot at start location

  /**************************** Step 3: Follow wall/corridore *******************************/
  while(!button_is_pressed(BUTTON));  // Wait till the user presses the button
  TURN_OFF_LED;
  delay(5000);  // wait 5 sec before moving the robot
  if(button_is_pressed(BUTTON) && (state == 1)) // button is pressed and the calibration was successful
  {
     follow_wall_duration(5000, RIGHT, 10.0, 600);  // follow the right side wall for 5 seconds maintating
                                                    // a distance of 10.0 cm from the wall.
  }

  /******************************************************************************************/

  /**************************** Step 4: Make a right/left turn ******************************/

  /******************************************************************************************/
}

 /**************************** End of program ******************************/

void loop() 
{

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
        orientation = get_orientation(RIGHT);
      }
      else if(dir == LEFT)
      {
        robot_deviation = get_avg_distance(LEFT);
        orientation = get_orientation(LEFT);
      }
      
    
      PID_ornt.Compute();
      PID_dev.Compute();
      set_robot_speed(DesiredVRobot - (int) (abs(PID_ornt_output) * 0.50) );
      int new_omega = (int) (0.7 * PID_ornt_output) + (0.3 * PID_dev_output);
      set_robot_omega(-1 * (int) new_omega);
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
