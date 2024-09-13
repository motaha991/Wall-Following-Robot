/******************************************************************************************************************* 
 *   This program is used to calibrate and test the range sensors mounted on Galvanizers 2.0. The robot has 5 range
 *   sensors (two on the left, two on the right and one at the front) to measure distances from the walls on each 
 *   side. This version performs the following tasks.
 *    
 *    1. Adds the calibration routine for the front sensor.
 *    2. Write function for measuring the distance from the wall.
 *    3. Write function for measuring the orientation (non-parallelism) of the robot.
 *    
 *    Author:   Muhammad Taha
 *    Dated:    26/01/2024
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

#define DEBUG_PRINTS

bool read_ranges_continous = false;
int continous_counter = 0;
      
void setup() 
{
  /************************* Step 1: Initiallize IOs ************************************/
  #ifdef DEBUG_PRINTS
    Serial.begin(115200);
    Serial.println("Starting the robot");
  #endif
  
  init_range_sensors();
  
  pinMode(RIGHT_BUTTON, INPUT_PULLUP); // This pin will read 0 if the button is pressed
  pinMode(LEFT_BUTTON, INPUT_PULLUP); // This pin will read 0 if the button is pressed
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT); 
  /******************************************************************************************/
}

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

    // Receive command "CalibrateSensors"
    if(temp.equals("CalibrateSensors"))
    {
      int err_code = calibrate_range_sensors();
      if(err_code == 0)
      {
        Serial.println("Calibration successful!");
        turn_LED_on(BLUE_LED);
        beep_buzzer();
        delay(2000);
        turn_LED_off(BLUE_LED);
        
      }
      else
      {
        Serial.print("Calibration failed! Error Code: ");
        Serial.println(err_code);
        turn_LED_on(RED_LED);
        beep_buzzer_times(3);
        delay(1000);
        turn_LED_off(RED_LED);
      }
    }
    // Receive command "LoadCalibration"
    if (temp.equals("LoadCalibration"))  /// If we want to load previously saved config
    {
      loadCalibration();
      Serial.println("Caliration loaded!");
    }

    // Receive command "RangesContinous". Will print out ranges for 5 seconds
    if(temp.equals("RangesContinous"))
    {
      read_ranges_continous = true;
      continous_counter = 0;
    }
    
  }// Serial.avaialble() ends here

  if((read_ranges_continous) && (millis()%100 == 0))
  {
    continous_counter ++;
    read_range_sensors();
    print_range_values_only();

    if(continous_counter == 30) // print ranges for 3 seconds
      read_ranges_continous = false;
  }
}
