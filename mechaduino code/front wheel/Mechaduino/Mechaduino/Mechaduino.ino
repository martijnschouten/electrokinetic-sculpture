 /*
  Author: Martijn Schouten
  Contact: https://github.com/martijnschouten/

  To use this program you first need to calibrate your mechaduino. To do so:
  - Upload the project in the for calibration folder
  - Go to the serial monitor and connect to your mechaduino
  - Start the calibration routine by sending c to your mechaduino. Make sure it is free to rotate.
  - Copy the outputted encoder lookup table to parameters.cpp

  To make this run stably on your hardware you may have to tune T_static, d_term (current file) and vKp, vKi, vKd (parameters.cpp)
*/

#include "Utils.h"
#include "Parameters.h"
#include "State.h"
#include "analogFastWrite.h"
#include <arduino-timer.h>


//////////////////////////////////////
/////////////////SETUP////////////////
//////////////////////////////////////

//Velocity calculation
float yw_old = 0;         //Last measured position
float v_loc = 0;          //Low pass filtered velocity estimate
float v_a = 0.7265;       //a coefficient of the low pass filter on the velocity
float v_b = 0.1367;       //b coefficient of the low pass filter on the velocity

//Acceleration calculation
float v_old = 0;          //Last calculated velocity
float a_loc = 0;          //Low pass filtered acceleration estimate
float a_a = 0.9691;       //a coefficient of the low pass filter on the acceleration
float a_b = 0.0155;       //b coefficient of the low pass filter on the acceleration

//Friction compensation
float T_static = 20;      //Estimate of the current needed to overcome the static friction in mA and divided by 8.6 
float v_idle = 0.04;      //Velocity above which the static friction compensation will become active
float p_term = 0.1;       //Determines the stiffness of the spring when rotated in the clockwise direction
float d_term = -10;       //Estimate of the dynamic friction
float m_term = 5000;      //Determines the additional moment of inertia

//Other variables
float set_point = 180;    //Will store the location where the wheel last turned in the counter-clockwise direction
int state = 0;            //state = 0 -> moving in the clockwise dirction, state = 1 -> moving in the counter-clockwise dirction
int timeout = 0;          //Before this variable reaches a certain treshhold, the output current will be kept zero in order to improve startup behaviour

//Variables used for switching between zen and interactive mode
int state2 = 0;           //state = 0 -> interactive mode. state = 1 -> zen mode
int latch_timer = 0;      //Timer for debouncing the button
int latch_timeout = 1000; //Timeout after which the button can be presed again (in ms)

Timer<1000, micros> timer;

bool control_loop(void *)
{
  //check if the mechaduino control loop isn't running
  if(mutex == true)
  {
      //if it is running quit this loop, to prevent lockup
      return true;
  }

  //calculate velocity, and low pass filter
  v_loc = v_a*v_loc + v_b*(yw-yw_old);
  a_loc = a_a*a_loc + a_b*(v_loc-v_old);

  if (state2 == 0 || state2 == 1)
  {
      //Check if the mode button has been pressed recently
      if (latch_timer < latch_timeout)
      {
        latch_timer++;
      }
      else
      {
        //if that's not the case, check if it is pressed and if so switch mode
        if (state2 == 0)
        {
          if (!digitalRead(2))
          {
             state2 = 1;
          }
        }
        else
        {
           if (digitalRead(2))
          {
             mode = 'v';
             state2 = 2;
             latch_timer = 0;
          }
        }
      }   
    
    //switch from moving with positive v to moving with negative v
    if ((state == 0) and(v_loc < -v_idle))
    {         
      state = 1;//switch to moving with negative v
    }
    //switch from moving with negative v to moving with positive v
    if ((state == 1) and (v_loc > v_idle))
    {         
      state = 0;//switch to moving with negative v
    }
  
    //apply non-linear equation
    if (yw < set_point)
    {
      r = -p_term*(yw-set_point)-d_term*v_loc-m_term*a_loc;
    }
    else
    {
       r = -d_term*v_loc-m_term*a_loc;
    }
    
    if ((state == 1) and (yw>set_point)){
        set_point = yw;//lock the setpoint
    }
  
  
    //static friction compensation
    if (state == 0)
    {
       r = r + T_static;
    }
    else 
    {
       r  = r - T_static;
    }

    //improve startup behaviour
    if (timeout<500)
    {
        r = 0;
        v_loc = 0;
        timeout++;
    }
  }
  else
  {
      //Check if the mode button has been pressed recently
      if (latch_timer < latch_timeout)
      {
        latch_timer++;
      }
      else
      {
        //if that's not the case, check if it is pressed and if so switch mode
        if (state2 == 2)
        {
          if (!digitalRead(2))
          {
             state2 = 3;
          }
        }
        else
        {
           if (digitalRead(2))
          {
             state2 = 0;
             latch_timer = 0;
             mode = 't';
          }
        }
      }

      //in zen mode, just move the both wheels slowly
      r = 2.5;
      v_loc = 0.025;
  }

  //Send the current torque out over usb for debug purposes
  SerialUSB.println(r);
  
  //Send the current velocity to the other mechaduino so it can mirror it
  Serial1.println(v_loc);

  //Store current values, for use in the low pass filters
  yw_old = yw;
  v_old = v_loc;
  return true;
}

void setup()        // This code runs once at startup
{
  u = 0;                        
  digitalWrite(ledPin,HIGH);        // turn LED on 
  setupPins();                      // configure pins
  setupTCInterrupts();              // configure controller interrupt

  SerialUSB.begin(115200);          // start debug serial connection
  Serial1.begin(115200);            // start serial connection with the other mechaduino
  delay(3000);                      // This delay seems to make it easier to establish a connection when the Mechaduino is configured to start in closed loop mode.  
  setupSPI();                       // Sets up SPI for communicating with encoder
  
  digitalWrite(ledPin,LOW);         // turn LED off 
  pinMode(2, INPUT_PULLUP);         // enable the pullup on the mode button

  
  // spot check some of the lookup table to decide if it has been filled in
  if (lookup[0] == 0 && lookup[128] == 0 && lookup[1024] == 0)
    SerialUSB.println("WARNING: Lookup table is empty! Run calibration");

  enableTCInterrupts();             // start in closed loop 
  mode = 't';                       // start in torque mode
  r = 0;                            // start with no torque
  
  timer.every(1, control_loop);     //run the control loop every time timer.tick() is called
}
  


//////////////////////////////////////
/////////////////LOOP/////////////////
//////////////////////////////////////


void loop()                 // main loop
{
  delayMicroseconds(1000);//necessary to give the control loops of the mechaduino time to run
  timer.tick();//run the control loop
}
