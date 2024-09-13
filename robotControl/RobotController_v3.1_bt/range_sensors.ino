#include <Wire.h>
#include <VL53L0X.h>
#include <EEPROM.h>

#define LEFT  0
#define RIGHT 1

#define RANGE_SENSORS_EEPROM_ADDRESS 0x00

#define TCAADDR 0x70
#define NUM_SENSORS 5

#define S1R 0     // Right-front sensor 
#define S2R 1     // Right-back sensor
#define S1L 2     // Left-front sensor
#define S2L 3     // Left-back sensor
#define S1F 4     // Front sensor

//String sensor_names[NUM_SENSORS] = {"S1R", "S2R", "S1L", "S2L", "S1F"};

uint8_t idx_arr[NUM_SENSORS] = {2, 3, 0, 1, 4};
unsigned int ranges[NUM_SENSORS] = {0};
unsigned int calib_offsets[NUM_SENSORS] = {0};


typedef VL53L0X* VL53L0XPtr;
VL53L0XPtr sensors[NUM_SENSORS];
  
void tcaselect(uint8_t i) 
{
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  int result = Wire.endTransmission();
}

int init_range_sensors(void)
{
  Wire.begin();
  Serial.println("starting...");
  for (uint8_t i = 0; i < NUM_SENSORS; i++)
  {
    tcaselect(idx_arr[i]);
    
    VL53L0XPtr sensor = new VL53L0X();
    sensor->init();
    sensor->setTimeout(100);
    sensor->setMeasurementTimingBudget(75000);  // Timing budget in Micro-Seconds
    sensor->startContinuous();
    sensors[i] = sensor;
  }
}

void read_range_sensors()
{
	
  for (uint8_t i = 0; i < NUM_SENSORS; i++)
  {  
    tcaselect(idx_arr[i]);

    VL53L0XPtr sensor = sensors[i];

    if(sensor->last_status == 0)
		{
      unsigned int temp =  sensor->readRangeContinuousMillimeters();
      if((temp - calib_offsets[i]) < 60000)
        ranges[i] = temp - calib_offsets[i];
      else
        ranges[i] = 0;
		}

    if (sensor->timeoutOccurred()) 
		  Serial.print(" [timeout]");
  }
}

int read_single_range_sensor(int sens_idx)
{
  tcaselect(idx_arr[sens_idx]);
  VL53L0XPtr sensor = sensors[sens_idx];

  if(sensor->last_status == 0)
  {
    unsigned int range = sensor->readRangeContinuousMillimeters() - calib_offsets[sens_idx];
    if(range > 60000)
      return(0);
    else
      return(range);
  }
  if(sensor->timeoutOccurred()) 
  {
    Serial.print("Sensor");
    Serial.print(sens_idx);
    Serial.println(" : timed out");
    return(-1);
  }
}


void loadCalibration(void)
{
  int eeAddress = RANGE_SENSORS_EEPROM_ADDRESS;
  for(int i=0; i<NUM_SENSORS; i++)
  {
    EEPROM.get(eeAddress, calib_offsets[i]);
    Serial.print("Sensor ");
    Serial.print(i);
    //Serial.print(sensor_names[i]);
    Serial.print(" offset: ");
    Serial.println(calib_offsets[i]);
    eeAddress += sizeof(unsigned int);
  }  
}

void print_range_sensors(void)
{
	Serial.println("");
	for(int i=0; i<NUM_SENSORS; i++)
	{
		Serial.print("Range at sensor ");
    Serial.print(i);
		//Serial.print(sensor_names[i]);
		Serial.print(": ");
		Serial.println(ranges[i]);
	}
	Serial.println("");
}

/*  This function returns the orientation of the robot w.r.t. the left or the right wall.
 *  It uses the difference between the distances of the front and back sensors on a 
 *  particular side to determine the orientation. 
 *  
 *  It uses a special case where if the direction argument is sent in as CORRIDORE, it 
 *  will return the average of both left and right orientations.
 */
int get_orientation(uint8_t dir)
{
  
  if(dir == RIGHT)
    return((ranges[S1R]) - (ranges[S2R])); // S1R - S2R
  else if(dir == LEFT)
    return((ranges[S2L]) - (ranges[S1L])); // S2L - S1L
  else
  {
    return(((ranges[S1R] - ranges[S2R])/2) + ((ranges[S2L] - ranges[S1L])/2)); //average of both left and right orientations
  }
}

/*  This function determines how far away from a set distance the robot is from
 *  the right or left side.It takes average of S1R/S1L and S2R/S2L for this 
 *  purpose. The inputs are direction (RIGHT or LEFT) and the distance to be 
 *  maintained from the wall (in mm).
 */
double get_range_error(unsigned char dir, int set_dist)
{
   double dist_err = 0;
   if(dir == RIGHT) // Use sensors S1R and S2R
   {
    dist_err = set_dist - ((ranges[S1R] + ranges[S2R]) / 20.0);  // Average of S1 and S2 subtracted from Set Point Distane set_dist
   }
   else // Use sensors S1L and S2L
   {
    dist_err = set_dist - ((ranges[S1L] + ranges[S2L]) / 20.0);  // Average of S1 and S2 subtracted from Set Point Distane set_dist
   }
   return(dist_err);
}

/*  This function takes as input a direction (left or right) and
 *  returns the average distance from the wall on that side. The
 *  distance returned is in centimeters.
 */
double get_avg_distance(unsigned char dir)
{
  double avg_dist = 0.0;
  /************* Removing this block so that only a single sensor is used for distance  
  if(dir == RIGHT) // Use sensors S1R and S2R
  {
    avg_dist = (ranges[S1R] + ranges[S2R])/20.0;
  }
  else
  {
    avg_dist = (ranges[S1L] + ranges[S2L])/20.0;
  }
  ************************************************************************************/
  if(dir == RIGHT) // Use sensors S1R and S2R
  {
    avg_dist = ranges[S1R]/10.0;
  }
  else
  {
    avg_dist = ranges[S1L]/10.0;
  }
  return(avg_dist);
}

/*  This function calculates the deviation of the robot from the center of the corridore. 
 *  The use case for this function is to drive the robot at an arbitrary distance from 
 *  the left or rigth wall in the corridore. If the robot is centered in the corridore 
 *  it will return 0. It will return a positive value if the robot deviates to the right
 *  of the center and will return a negative value if the robot deviates to the left of 
 *  the center of the corridore.
 */
int get_dist_deviation(void)
{
  int dist_left = get_avg_distance(LEFT);
  int dist_right = get_avg_distance(RIGHT);
  return(dist_right - dist_left);
}

int dist_at_front(void)
{
  return(ranges[S1F]);
}
