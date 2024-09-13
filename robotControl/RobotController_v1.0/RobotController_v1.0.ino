/******************************************************************************************************************* 
 *   This program is the main control program for making a differential robot follow a wall. The robot has 4 range
 *   sensors (two on the left and two on the right) to measure distances from the walls on each side. This version 
 *   achieves the following tasks.
 *    1. Write and test functions for setting the robot linear and angular velocities.
 *    2. Write functiond for measuring the distance from the wall.
 *    3. Write function for measuring the amount of deviation (non-parallelism in the robot.
 *    4. Write functions for computing errors.
 *    5. Write code for implementing PID controller for wall following.
 *    
 *    Author: Muhammad Taha
 *    Dates:  17/07/2023
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
double current_deviation;   // Input to myPID
double prev_deviation = 0;
double PID_output;                  // Output of myPID
double reference_distance = 10.0;   // Setpoint: desired distance (in cm) from the wall
double reference_deviation;         // This would be 0 for the reference distance from the wall
double deviation_error;             // difference between current_deviation and the reference deviation
int orientation_error;

int DesiredVRobot = 0;  // This is the desired linear velocity if the robot correctly positioned
int state = 0;

//Specify the links and initial tuning parameters
//double Kp = 50.5, Ki = 10.0, Kd = 25.0;//15.5;
double Kp = 20.5, Ki = 0.0, Kd = 9.5;  //15.5;
int SampleTime = 100;

PID myPID(&current_deviation, &PID_output, &reference_deviation, Kp, Ki, Kd, DIRECT);

void PID_initiallize()
{
  myPID.SetSampleTime(SampleTime);
  myPID.SetOutputLimits(-250.0,250.0);  // 25% of the robot's Motor velocities
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
      myPID.SetMode(MANUAL);
      DesiredVRobot = 0;
      ramp_robot_speed(0);
      ramp_robot_omega(0);
      stop_robot();
      Serial.println("Aborting.");
    }

    if((inbound.fullMatch("FollowWall")) && (state == 1))
    {
      state = 2;
      reference_distance = 10.0;  // Distance from the wall
      reference_deviation = 0.0;
      DesiredVRobot = 600;
      myPID.SetMode(AUTOMATIC);
    }
  
  }

  /*************************************************************************************************************/
  
  
  if((state == 2) && ((millis()%100) == 0))  // The robot should follow line
  {
    //unsigned long prev_t = millis();
    read_range_sensors();  // Read Sensors
    current_deviation = get_range_error(RIGHT, reference_distance);
    //current_distance = (double) get_avg_distance(RIGHT);
    orientation_error = get_orientation_error(RIGHT);
    myPID.Compute();
    
    if(abs(orientation_error) < 25)
    {
      set_robot_speed(DesiredVRobot - (int) (abs(PID_output) * 0.50) );
      set_robot_omega((int) PID_output);
    }
    else
    {
      set_robot_speed(DesiredVRobot - (abs(orientation_error) * 2.50));
      set_robot_omega(orientation_error * 2.5);
    }
    //Serial.print("Comp_t: ");
    //Serial.print(millis() - prev_t);
    Serial.print("Orn Err: ");
    Serial.print(orientation_error);
    Serial.print(" Dev: ");
    Serial.print(current_deviation);
    Serial.print(" Vrob: ");
    Serial.print(get_robot_speed());
    Serial.print(" Omega: ");
    Serial.println(get_robot_omega());  
    prev_deviation = current_deviation; 
  }

  /*Serial.print(50);
  Serial.print(" ");
  Serial.print(300);
  Serial.print(" ");
  Serial.println(current_distance);
  */

}
