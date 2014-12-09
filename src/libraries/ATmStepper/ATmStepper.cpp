/*
ATM STEPPER C 2009 Kieran Levin DEC 28 2009
 */



#include "Arduino.h"
#include "ATmStepper.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//update these to initialize a new speed
	 volatile int velocity1x;
	 volatile int velocity1y;

	 volatile byte TNS1x;
	 volatile byte TNS1y;
	 volatile byte running; 
	// update these variables to initialize a new position 
	 volatile unsigned int posnew1x; 
	 volatile unsigned int posnew1y; 
	// these store our current position - do not change these to prevent damage to the head
	 volatile unsigned int pos1x;
	 volatile unsigned int pos1y; 

    	// motor pin numbers:
    	 int _motorx_pin;
    	 int _motory_pin;
	 int _motor_dir;
	 int _motor_enable; 
	 int _x_lim;
	 int _y_lim;
	unsigned long _step_freq_max;

#define FORWARD	HIGH
#define REVERSE LOW
//////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * two-wire constructor.
 * Sets which wires should control the motor.
 */
void ATmStepper::ATmStepperInit(unsigned long step_freq_max, int motor1_pin, int motor2_pin, int motor_direction, int motor_enable, int x_lim, int y_lim)
{
  velocity1x = 0;      //set initial velocity
  velocity1y = 0;      //set initial velocity

  posnew1x = 0; 
  posnew1y = 0; 

  pos1x = 0;
  pos1y = 0;

  running = 0;

  // Arduino pins for the motor control connection:
  _motorx_pin = motor1_pin;
  _motory_pin = motor2_pin;

  _motor_enable = motor_enable;
  _motor_dir = motor_direction; 
  _x_lim = x_lim;
  _y_lim = y_lim;

  _step_freq_max = step_freq_max;

  // setup the pins on the microcontroller:
  pinMode(_motorx_pin, OUTPUT);
  pinMode(_motory_pin, OUTPUT);
  pinMode(_motor_dir, OUTPUT); 
  pinMode(_motor_enable, OUTPUT);

  pinMode(_x_lim, INPUT);
  pinMode(_y_lim, INPUT);
  digitalWrite(_x_lim, HIGH);	//enable pullups
  digitalWrite(_y_lim, HIGH);
  //init our timer to run at the max step speed (1000hz max) and to call the function step each time
  MsTimer2::set(step_freq_max, ATmStepper::step);
  //start the hardware timer
  delay(30);
	findzero(); 


}
/* 

*/ 
void ATmStepper::setvelocity1x(int vel1x)
{
	
	if(0 != vel1x)
	{
		posnew1x = 0;
		velocity1x = vel1x; 
		if (0 == TNS1x)
			TNS1x = byte(abs(velocity1x));
		run();
	}
	else if(0 == posnew1x)
		velocity1x = vel1x; 
}
/* 

*/ 
void ATmStepper::setvelocity1y(int vel1y)
{
	
	if(0 != vel1y)
	{
		posnew1y = 0; // if we are doing an update velocity command disable position 
		velocity1y = vel1y; 
		if (0 == TNS1y)
			TNS1y = byte(abs(velocity1y));
		run();
	}
	else if(0 == posnew1y)
		velocity1y = vel1y; 
}
/* 

*/ 
void ATmStepper::setpos(unsigned int npos1x, unsigned int npos1y)
{
	posnew1x = npos1x;	
	if (npos1x > pos1x) 
		velocity1x = 1; 
	else if (npos1x < pos1x)
		velocity1x = -1; 

	TNS1x = 1;

	posnew1y = npos1y;
	if (npos1y > pos1y) 
		velocity1y = 1; 
	else if (npos1y < pos1y)
		velocity1y = -1; 

	TNS1y = 1;
	run();
}
/* 

*/ 
unsigned int ATmStepper::getpos1x()
{
	return pos1x; 
}
unsigned int ATmStepper::getpos1y() 
{
	return pos1y; 
}
/* 

*/ 
void ATmStepper::findzero()
{
	pos1x = XMAX - 1; 
	velocity1x = -1; 
	TNS1x = 1; 
	posnew1x = 1;
	
	pos1y = YMAX - 1; 
	velocity1y = -1; 
	TNS1y = 1; 
	posnew1y = 1;
	run();
	while(1 != pos1x || 1 != pos1y); //busywait till we have found zero
	Serial.println("\tOK");	
}
/* 
	
*/ 
void ATmStepper::run()
{
	if (0 == running)
	{
	digitalWrite(_motor_enable, LOW);   	
	MsTimer2::start();	
	running  = 1;
	}
}
/* 

*/ 
void ATmStepper::stop() 
{
	MsTimer2::stop();
	digitalWrite(_motor_enable, HIGH); 
	running = 0;
}
/*
  Moves the motor steps_to_move steps.  If the number is negative, 
   the motor moves in the reverse direction.
 */
void ATmStepper::step()
{  
	if( 1 < TNS1x) 		// if we have not reached the next trigger time then decrement 
		TNS1x--;	
	else if (1 == TNS1x)	//we have triggered the next step 
	{
		
		
		if (0 < velocity1x)	// velocity is positive - update position and motor 
		{

			if(XMAX > pos1x) 
			{
				updatemotor(_motorx_pin, FORWARD);
				pos1x++;
			}
		}
		else if (0 > velocity1x)	//velocity is negative - update position and motor 
		{

			if(0 != pos1x)
			{
				updatemotor(_motorx_pin, REVERSE);
				pos1x--;
			}
			if(LOW == digitalRead(_x_lim)) 
			{
				//Serial.println("\tXLIM");	
				delayMicroseconds(10383);
				while(LOW == digitalRead(_x_lim))
				{
					updatemotor(_motorx_pin, FORWARD);
					delayMicroseconds(10383);
				}
				pos1x = 1;
				//TODO FIXME RESET MOTOR DRIVER HERE TO SET POSITION TO ZERO
				updatemotor(_motorx_pin, FORWARD);
			}
		}
	

		if(pos1x == posnew1x && 0 != posnew1x)	// if our position has reached the desired positon and we are in position mode then 
		{					// no more stepping 
			velocity1x = 0; 
			posnew1x = 0;
			TNS1x = 0; // turn off this motor 
		}
		else 
			TNS1x = byte(abs(velocity1x));	//update our step counter to fire for the next step
	
	}
	///////////////////////////////////// update for the Y axis 
	if( 1 < TNS1y) 		// if we have not reached the next trigger time then decrement 
		TNS1y--;	
	else if (1 == TNS1y)	//we have triggered the next step 
	{
		
		
		if (0 < velocity1y)	// velocity is positive - update position and motor 
		{
			if(YMAX > pos1y)
			{
				updatemotor(_motory_pin, FORWARD);
				pos1y++;
				}
		}
		else if (0 > velocity1y)	//velocity is negative - update position and motor 
		{
			if (0 != pos1y)
			{
				updatemotor(_motory_pin, REVERSE);
				pos1y--;
			}
			// check for our limit switch
			if(LOW == digitalRead(_y_lim)) 
			{
				delayMicroseconds(10383);
				while(LOW == digitalRead(_y_lim))
				{
					updatemotor(_motory_pin, FORWARD);
					delayMicroseconds(10383);
				}
				pos1y = 1;
				//TODO FIXME RESET MOTOR DRIVER HERE TO SET POSITION TO ZERO
				updatemotor(_motory_pin, FORWARD);
			}
		}
	

		if(pos1y == posnew1y && 0 != posnew1y)	// if our position has reached the desired positon and we are in position mode then 
		{					// no more stepping 
			velocity1y = 0; 
			posnew1y = 0;
			TNS1y = 0; // turn off this motor 
		}
		else 
			TNS1y = byte(abs(velocity1y));	//update our step counter to fire for the next step
	
	}

	if((0 == TNS1x) && (0 == TNS1y)) //disable the motors and timer as we are not stepping 
		stop();

	// check for limits


}

/*
 * Moves the motor forward or backwards.
 */
void ATmStepper::updatemotor(int motor, int direction)
{
	//set up the direction pin 
	digitalWrite(_motor_dir, direction);
	//pulse the corresponding step pin to perform a step 
	digitalWrite(motor, HIGH);
	digitalWrite(motor, LOW);
}

