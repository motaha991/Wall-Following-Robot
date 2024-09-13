/******************************************************************************************************************* 
 *   This program is the main control program for making a differential robot follow a wall. The robot has 4 range
 *   sensors (two on the left and two on the right) to measure distances from the walls on each side. This version 
 *   achieves the following tasks.
 *    1. Write and test functions for setting the robot linear and angular velocities.
 *    2. Write functiond for measuring the distance from the wall.
 *    3. Write function for measuring the amount of deviation (non-parallelism in the robot.
 *    4. Write functions for computing errors.
 *    5. Write code for implementing PID controller for wall following.
 *        - Two PID controllers will be used.
 *        - First will correct the distance error
 *        - Second will correct the orientation error
 *    
 *    Author:   Muhammad Taha
 *    Dated:    26/07/2023
 *    version:  2.0
 *    ************************************************************************************************************/
#define LEFT  0
#define RIGHT 1

#include <PID_v1.h>
#define PI_CONTROL_ON     //choose at compile time whether to use PID or not    

/************************** Library for Messaging ********************************/
 // Include AsciiMassageParser.h and AsciiMassagePacker.h for ASCII format massage packing and parsing.
#include <AsciiMassageParser.h>

// Instantiate an AsciiMassageParser and an AsciiMassagePacker.
AsciiMassageParser inbound;

/*********************************************************************************/

/************************ PID Definitions and Variables **************************/
// defines variables
double dist_from_wall;               // Distance from the wall. Input to PID_dist
double PID_dist_output;             // Output of PID_dist
double reference_distance = 10.0;   // Setpoint: desired distance (in cm) from the wall

double orientation;                 // Measured orientation from the wall
double reference_orientation = 0;   // Setpoint: This would be 0 for when the robot is parallel to the wall
double PID_ornt_output;             // Output of PID_ornt

int DesiredVRobot = 0;              // This is the desired linear velocity if the robot correctly positioned
int state = 0;

// The gains for the two PID controllers   

double Kp_dist = 2.5, Ki_dist = 0.0, Kd_dist = 0.05;
double Kp_ornt = 2.5, Ki_ornt = 1.25, Kd_ornt = 0.05;


int SampleTime = 100;

PID PID_dist(&dist_from_wall, &PID_dist_output, &reference_distance, Kp_dist, Ki_dist, Kd_dist, DIRECT);
PID PID_ornt(&orientation, &PID_ornt_output, &reference_orientation, Kp_ornt, Ki_ornt, Kd_ornt, DIRECT);

void PID_initiallize()
{
  PID_dist.SetSampleTime(SampleTime);
  PID_dist.SetOutputLimits(-250.0,250.0);  // 25% of the robot's Motor velocities

  PID_ornt.SetSampleTime(SampleTime);
  PID_ornt.SetOutputLimits(-250.0,250.0);  // 25% of the robot's Motor velocities
}

void setup() 
{
  Serial.begin(115200);
  init_range_sensors();
  Serial.println("Starting the robot");
  initiallize_timers();
  stop_robot();
  PID_initiallize();
}

void loop() 
{
  /**************** Parse incoming messages and set appropriate flags **************************/
  if(inbound.parseStream( &Serial)) 
  {
     // Receive command "GetRanges"
    if(inbound.fullMatch("GetRanges"))
    {
      read_range_sensors();
      print_range_sensors();      
    }
    // Receive command "CalibrateSensors"
    if(inbound.fullMatch("CalibrateSensors"))
    {
      int err_code = calibrate_range_sensors();
      if(err_code == 0)
        Serial.println("Calibration successful!");
      else
      {
        Serial.print("Calibration failed! Error Code: ");
        Serial.println(err_code);
      }
    }
    // Receive command "LoadCalibration"
    if ((inbound.fullMatch("LoadCalibration"))  && (state == 0))  /// If we want to load previously saved config
    {
      state = 1;
      loadCalibration();
      Serial.println("Caliration loaded!");
    }
    // This command will move the robot forward, then backwards, rotate left and then rotate right before stopping
    if((inbound.fullMatch("TestRobotMotion")) && (state == 0))
    {
      Serial.println("Testing robot motion!");
      Serial.println("Moving forward and right.");
      ramp_robot_speed(600);
      ramp_robot_omega(50);
      delay(2000);
      ramp_robot_speed(0);
      ramp_robot_omega(0);
      stop_robot();
      Serial.println("Moving forward and left.");
      delay(5000);
      ramp_robot_speed(600);
      ramp_robot_omega(-50);
      delay(2000);
      ramp_robot_speed(0);
      ramp_robot_omega(0);
      stop_robot();
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

    if((inbound.fullMatch("Abort")))
    {
      state = 0;
      PID_ornt.SetMode(MANUAL);
      DesiredVRobot = 0;
      ramp_robot_speed(0);
      ramp_robot_omega(0);
      stop_robot();
      Serial.println("Aborting.");
    }

    if((inbound.fullMatch("FollowWall")) && (state == 1))
    {
      state = 2;
      reference_distance = 10.0;    // Required distance from the wall
      reference_orientation = 0.0;  // Required orientation
      DesiredVRobot = 600;
      PID_dist.SetMode(AUTOMATIC);
      PID_ornt.SetMode(AUTOMATIC);
    }
  
  }

  /*************************************************************************************************************/
  
  
  if((state == 2) && ((millis()%100) == 0))  // The robot should follow wall
  {
    read_range_sensors();  // Read Sensors
    //current_deviation = get_range_error(RIGHT, reference_distance);
    dist_from_wall = (double) get_avg_distance(RIGHT);
    orientation = get_orientation(RIGHT);
    
    PID_ornt.Compute();
    PID_dist.Compute();
    set_robot_speed(DesiredVRobot - (int) (abs(PID_ornt_output) * 0.50) );
    int new_omega = (int) (0.7 * PID_ornt_output) + (0.3 * PID_dist_output);
    set_robot_omega(-1 * (int) new_omega);
    
    Serial.print("Orn Err: ");
    Serial.print(orientation);
    Serial.print(" Dist: ");
    Serial.print(dist_from_wall);
    Serial.print(" Vrob: ");
    Serial.print(get_robot_speed());
    Serial.print(" Omega: ");
    Serial.println(get_robot_omega());  
  }

  /*Serial.print(50);
  Serial.print(" ");
  Serial.print(300);
  Serial.print(" ");
  Serial.println(current_distance);
  */

}
