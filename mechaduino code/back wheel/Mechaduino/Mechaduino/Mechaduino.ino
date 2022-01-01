 /*
  Author: Martijn Schouten
  Contact: https://github.com/martijnschouten/

  To use this program you first need to calibrate your mechaduino. To do so:
  - Upload the project in the for calibration folder
  - Go to the serial monitor and connect to your mechaduino
  - Start the calibration routine by sending c to your mechaduino. Make sure it is free to rotate.
  - Copy the outputted encoder lookup table to parameters.cpp

  To make this run stably on your hardware you may have to tune  vKp, vKi, vKd (parameters.cpp)
*/

#include "Utils.h"
#include "Parameters.h"
#include "State.h"
#include "analogFastWrite.h"

//////////////////////////////////////
/////////////////SETUP////////////////
//////////////////////////////////////

float deadzone = 2;//Any velocity below this velocity will be considered to zero

void setup()        // This code runs once at startup
{                         
   
  digitalWrite(ledPin,HIGH);        // turn LED on 
  setupPins();                      // configure pins
  setupTCInterrupts();              // configure controller interrupt

  SerialUSB.begin(115200);
  Serial1.begin(115200);         
  delay(3000);                      // This delay seems to make it easier to establish a connection when the Mechaduino is configured to start in closed loop mode.  
  //serialMenu();                   // Prints menu to serial monitor
  setupSPI();                       // Sets up SPI for communicating with encoder
  digitalWrite(ledPin,LOW);         // turn LED off 
  
  // spot check some of the lookup table to decide if it has been filled in
  if (lookup[0] == 0 && lookup[128] == 0 && lookup[1024] == 0)
    SerialUSB.println("WARNING: Lookup table is empty! Run calibration");

  enableTCInterrupts();             // start in closed loop 
  mode = 'v';                       // start in velocity mode
  
}
  


//////////////////////////////////////
/////////////////LOOP/////////////////
//////////////////////////////////////


void loop()                 // main loop
{
  // Read the velocity that the other mechaduino send us
  String line;
  line = Serial1.readStringUntil('\n');

  // Convert the velocity from the other mechaduino to the unit desired by the mechaduino control loop
  r =  line.toFloat()*100;

  // Put the desired velocity to zero if it is below a certain value.
  if ((r < deadzone) && (r > -deadzone))
  {
    r = 0;
  }
}
