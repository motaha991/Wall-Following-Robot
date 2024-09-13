/*
  Software serial multple serial test

 Receives from the hardware serial, sends to software serial.
 Receives from software serial, sends to hardware serial.

 The circuit:
 * RX is digital pin 10 (connect to TX of other device)
 * TX is digital pin 11 (connect to RX of other device)

 Note:
 Not all pins on the Mega and Mega 2560 support change interrupts,
 so only the following can be used for RX:
 10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69

 Not all pins on the Leonardo support change interrupts,
 so only the following can be used for RX:
 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).



 */
#define use_soft_serial
#ifdef use_soft_serial

#include <SoftwareSerial.h>
SoftwareSerial mySerial(5, 4); // RX, TX
#endif

void setup() 
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) 
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.println("Goodnight moon!");

  #ifdef use_soft_serial
    // set the data rate for the SoftwareSerial port
    mySerial.begin(38400);
    //mySerial.begin(9600);
 #else
  Serial2.begin(38400);
  //Serial2.begin(9600);
 #endif
}

void loop() 
{ // run over and over

  #ifdef use_soft_serial
    if (mySerial.available())
    {
      Serial.write(mySerial.read());
    }
    if (Serial.available()) 
    {
      mySerial.write(Serial.read());
    }
  #else
  
    if (Serial2.available()) 
    {
      Serial.write(Serial2.read());
    }
    if (Serial.available())
     {
      Serial2.write(Serial.read());
    }
   #endif
  
}
