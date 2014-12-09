/*
ATM STEPPER C 2009 Kieran Levin 
Ver:1.1A Dec 28 2009
  Step C0 C1 C2 C3
     1  1  0  1  0
     2  0  1  1  0
     3  0  1  0  1
     4  1  0  0  1

  The sequence of controls signals for 2 control wires is as follows
  (columns C1 and C2 from above):

  Step C0 C1
     1  0  1
     2  1  1
     3  1  0
     4  0  0

  The circuits can be found at 
  http://www.arduino.cc/en/Tutorial/Stepper
*/

// ensure this library description is only included once
#ifndef ATmStepper_h
#define ATmStepper_h
//We need this library!!!! 
#include "MsTimer2.h"

// library interface description
namespace ATmStepper {
	#define XMAX	11000
	#define YMAX	7000
	void ATmStepperInit(unsigned long step_freq_max, int motor1_pin, int motor2_pin, int motor_direction, int motor_enable, int x_lim, int y_lim);
	void setvelocity1x(int vel1x);
	void setvelocity1y(int vel1y);

	void setpos(unsigned int npos1x, unsigned int npos1y); 

	unsigned int getpos1x(); 
	unsigned int getpos1y(); 

	void findzero(); 
	void run();
	void stop(); 

	 void step();
	 void updatemotor(int motor, int direction);

/*	//update these to initialize a new speed
	 volatile int velocity1x;
	 volatile int velocity1y;

	 volatile byte TNS1x;
	 volatile byte TNS1y;

	// update these variables to initialize a new position 
	 volatile unsigned int posnew1x; 
	 volatile unsigned int posnew1y; 

	 volatile unsigned int pos1x;
	 volatile unsigned int pos1y; 

    	// motor pin numbers:
    	 int _motorx_pin_1;
    	 int _motorx_pin_2;
    	 int _motory_pin_1;
    	 int _motory_pin_2;
	 int _motor_enable; 
	 int _x_lim;
	 int _y_lim;
	unsigned long _step_freq_max;
*/
};

#endif

