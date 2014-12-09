/*  Pan tilt controller library (EX1Pantilt.pde)
	Written by: Kieran Levin Dec 13th, 2011
	
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
//TODO FIX VELOCITY RAMPING FUNCTIONS IN ATMSTEPPER.CPP 
#define VERSION  "Pan Tilt V1.3B C 2012 Kieran Levin"
#include <Spi.h>
#include <Psx_analog.h>                                          // Includes the Psx Library 

#include <ATmStepper.h>

#include <EEPROM.h>

#include <Messenger.h>

//#include <Servo.h>
#include <ServoTimeTimer1.h>
////////////////////////////////////
//Motor Definition 
//was 5
 #define StepFreqMax 5
//#define StepFreqMax 3

#define PinTiltLim 8
#define PinPanLim 7

#define PinMotorEnable 16
#define PinMotorDir 14
#define PinMotorPanStep 15
#define PinMotorTiltStep 17


/////////////////////////////////////
//Messenger definition 
Messenger message = Messenger(); 

////////////////////////////////////
//Servo definition 
#define PinServo 9
//Servo servFocus;
ServoTimeTimer1 servFocus;
uint16_t serv_cal;
///////////////////////////////////
//Controller pins definition 
#define PinData 12
#define PinCmnd 11
#define PinAtt 10
#define PinClock 13

Psx Psx;                                                  // Initializes the playstation library 
////////////////////////////////////
//Camera pins definition 
#define PinZoom  6 
#define PinRec 5
#define PinMode 4

#define CENTER  0x80
//nomalized x^3 curve to allow finer resolution control for lower values....
/*
unsigned char modifier[128] = 
{
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,2,2,2,3,3,3,4,4,4,5,5,6,
6,7,7,8,9,9,10,11,12,12,13,14,15,16,17,18,19,20,21,23,24,25,26,28,29,31,32,34,
35,37,39,40,42,44,46,48,50,52,54,56,59,61,63,66,68,71,73,76,79,81,84,87,90,93,
96,100,103,106,110,113,117,120,124,128,132,136,140,144,148,152,156,161,165,170,
174,179,184,189,194,199,204,209,215,220,226,231,237,243,249,255};
*/
//offset value for the zoom centering algorithm 
int offset;
//general forloop counter 
int i;

char pressed; 

// bool to keep track if the ptz controls are flipped 
char flipped; 
// hold the preset positions 
unsigned int presets[4][2]; 
#define EEPROMVALID 0
#define EEPROMVALIDVAL 0xB0
#define EEPROMCENTER 1
#define EEPROMFLIP  2
#define EEPROMSERCAL 4
#define EEPROMMEMSTART 6
// converts the raw psx output (0 to 255) to my velocity commands 1 being the fastest
int psxtovelocity(byte raw)
{
   if (raw > 230)   return (1);
   if (raw > 204)   return (2);
   if (raw > 194)   return (3);
   if (raw > 184)   return (4);
   if (raw > 174)   return (5);
   if (raw > 164)   return (6);
   if (raw > 154)   return (7);     
   if (raw > 144)   return (8);
   if (raw > 112)   return (0);
   if (raw > 102)   return (-8);     
   if (raw > 92)    return (-7);
   if (raw > 82)    return (-6);
   if (raw > 72)    return (-5);
   if (raw > 62)    return (-4);
   if (raw > 52)    return (-3);
   if (raw > 25)    return (-2);

                    return (-1);   
     
}
// handles the zoom and zoom center offset 
byte zoommodifier(byte raw)
{
  int temp;

  temp = (offset + int(raw));   // reverse the direction 
  //bounds check.... 
  if(temp < 0) temp = 0; 
  if (temp > 255) temp = 255; 
 return byte(temp);   
}
// save all NVRAM data 
void savePresets()
{  
  noInterrupts();
                  EEPROM.write(EEPROMCENTER,byte(offset));
                  EEPROM.write(EEPROMFLIP,flipped); 
                  EEPROM.write(EEPROMSERCAL, serv_cal & 0x00FF);
                  EEPROM.write(EEPROMSERCAL+1, serv_cal >> 8);
                  for(i = 0; i < 4; i++)
                  {
                     EEPROM.write(EEPROMMEMSTART + 0 + (i << 2),byte(presets[i][0] & 0xFF));
                     EEPROM.write(EEPROMMEMSTART + 1 + (i << 2),byte(presets[i][0] >> 8));
                     EEPROM.write(EEPROMMEMSTART + 2 + (i << 2),byte(presets[i][1] & 0xFF));
                     EEPROM.write(EEPROMMEMSTART + 3 + (i << 2),byte(presets[i][1] >> 8));

                  }  
  interrupts();
}
//////////////////////////////////////////////////
// Updates the focus servo 
//#define POSMAX 3000
//#define POSMIN  1750
#define POSMIN  865 //not normal but for one head yes
//#define POSBACK  15
//uint16_t servPosition; 

//byte servPosFlags; 
//byte servPosCounter;
void updateFocus(byte raw)
{
  if ((raw > 148) || (raw < 108))
  servFocus.write(serv_cal + (raw >> 2)); 
  else 
  servFocus.write(serv_cal + 0x20); 
}
// prints help on the serial interface  (does not have enough program mem window to work...)
//void serialHelp()
//{
//      Serial.println(VERSION);
//      Serial.println("Help: ");   
//      Serial.println("Command Syntax = Char Value ... Value Value (Carrage return)");  
//      Serial.println("Move to position   : 'M 100 100'  ");  
//      Serial.println("Set X velocity     : 'X 1' or 'X -1'");  
//      Serial.println("Set Y velocity     : 'Y 1' or 'Y -1'");  
//      Serial.println("Return Coordinates : 'C' Returns X Y ");  
//      Serial.println("Run Pan tilt head  : 'G'");    
//      Serial.println("Stop Pan tilt head : 'S'");     
//      Serial.println("Camera Record      : 'R'");   
//      Serial.println("Zoom               : 'R Zval(0 to 255)'");
//      Serial.println("Help (this screen) : 'h or H'");         
//}
// handles all serial messages when they are ready 
void messageReady() 
{
  switch (message.readChar())
  {
    case 'M': //move to position
      if(message.available()) 
      ATmStepper::setpos(message.readInt(), message.readInt()); 
      break;
    case 'X': // set X velocity 
      if(message.available()) 
      ATmStepper::setvelocity1x(message.readInt());
      break; 
    case 'Y': // set Y velocity 
      if(message.available()) 
      ATmStepper::setvelocity1y(message.readInt());
      break;   
    case 'C': // get Coordinates (returns in form X Y)  
        Serial.print(ATmStepper::getpos1x(), DEC);
        Serial.print(' '); 
        Serial.print(ATmStepper::getpos1y(), DEC);
        Serial.println("  "); 
      break; 
    case 'G': // Run head 
      ATmStepper::run(); 
      break; 
    case 'S': // Stop head 
      ATmStepper::stop(); 
      break; 
    case 'R': // trigger record 
      digitalWrite(PinRec, HIGH);
      delay(10);
      digitalWrite(PinRec, LOW);
      break; 
    case 'Z': // operate zoom 
      analogWrite(PinZoom, byte(message.readInt() + offset));
      break; 
    case 'F': //operate focus 
      servFocus.write(message.readInt()); 
      break; 
    case 'h': 
    case 'H':// show help 
      Serial.println("Help: www.kieranlevin.com"); 
      break;     
    default: 
      Serial.println("Invalid"); 
      break; 
  }
      
       // Loop through all the available elements of the message
       while ( message.available() ) 
       {
	  // There must be some garbage at the end of the message
          message.readInt();
          Serial.println("Garbage in message Detected"); 
       }
       Serial.println("OK"); 
}
// initialize the controller head 
void setup()
{
  //delay(10);
  Serial.begin(9600); 
  Serial.println(VERSION);
  Serial.println("Please wait..."); 
  ///////////////////////////////////////////
  Serial.println("Initializing..."); 
  Serial.println("\tHead...");
  delay(100);
  ATmStepper::ATmStepperInit(StepFreqMax,PinMotorPanStep,PinMotorTiltStep,PinMotorDir,PinMotorEnable,PinPanLim,PinTiltLim); 
  Serial.println("\tCentering Head...");
  //TmStepper::setpos(7000,5000);
  ATmStepper::setpos(70,50);
  delay(300);
  /////////////////////////////////////////////////////
  Serial.println("\tNVRAM...");   
  pressed = 0; 
  if(EEPROMVALIDVAL == EEPROM.read(EEPROMVALID))
  { 
    offset = int(char(EEPROM.read(EEPROMCENTER)));
    flipped = EEPROM.read(EEPROMFLIP); 
    serv_cal = EEPROM.read(EEPROMSERCAL) | (EEPROM.read(EEPROMSERCAL + 1) << 8);;
    for(i = 0; i < 4; i++)
    {
       presets[i][0] = int(EEPROM.read(EEPROMMEMSTART + 0 + (i << 2))) | ( int(EEPROM.read(EEPROMMEMSTART + 1 + (i << 2))) << 8);
       presets[i][1] = int(EEPROM.read(EEPROMMEMSTART + 2 + (i << 2))) | ( int(EEPROM.read(EEPROMMEMSTART + 3 + (i << 2))) << 8);
    }
  }
  else
  {
    Serial.println("\tInvalid NVRAM...Defaults Loaded");   
    EEPROM.write(EEPROMCENTER, 0);
    EEPROM.write(EEPROMFLIP, 0); 
    EEPROM.write(EEPROMVALID, EEPROMVALIDVAL); 
    EEPROM.write(EEPROMSERCAL,byte(POSMIN  & 0x00FF));
    EEPROM.write(EEPROMSERCAL+1, byte(POSMIN >> 8));
    
    
  }
  pinMode(PinRec,OUTPUT);
  pinMode(PinMode,OUTPUT);
  pinMode(PinZoom,OUTPUT);
  digitalWrite(PinRec, LOW);
  digitalWrite(PinMode, LOW);
  analogWrite(PinZoom, byte(offset));
  ////////////////////////////////////////
  Serial.println("\tPSX...");   

  Psx.setupPins(PinData, PinCmnd, PinAtt, PinClock);  // Defines what each pin is used
                                                      // (Data Pin #, Cmnd Pin #, Att Pin #, Clk Pin #)
  delayMicroseconds(10);                                
  noInterrupts();
  Psx.initcontroller(psxAnalog);    
  interrupts();
  delayMicroseconds(10);  


  ///////////////////////////////////////////
  Serial.println("\tSerial Com..."); 
  message.attach(messageReady);
  ///////////////////////////////////////////

  servFocus.attach(9); 
  servFocus.write(serv_cal + 0x20);

  Serial.println("Initialization complete."); 

}


  

void loop()
{
  
  
  Psx.poll();					// update controller values 
  //if there is no controller connected... handle serial commands...
  if(0xFF == Psx.Controller_mode)
  {
    //handle any serial commands that may come our way...
    while ( Serial.available() )  message.process(Serial.read () );
     
  }
  // only update if the controller is connected and in the analog mode
  else if(0x70 == (Psx.Controller_mode & 0xF0))
  {
          // handle updating the controller speed from data read from position indicator...
	  if(flipped)
	  {
	    ATmStepper::setvelocity1x(-psxtovelocity(Psx.Left_x));  
	    ATmStepper::setvelocity1y(psxtovelocity(Psx.Left_y));
	  }
	  else 
	  {
	    ATmStepper::setvelocity1x(psxtovelocity(Psx.Left_x));  
	    ATmStepper::setvelocity1y(-psxtovelocity(Psx.Left_y));    
	  }

	  // output analog values to correct pins 
          if (Psx.Right_y > 148 || Psx.Right_y < 108){
	    analogWrite(PinZoom, zoommodifier(Psx.Right_y));
          }
          else {
            analogWrite(PinZoom, zoommodifier(0x80));
          }
          updateFocus(Psx.Right_x); 

	  // Handle any button press here
	  if (Psx.digital_buttons & (psxR1))
	    digitalWrite(PinRec, HIGH);
	  else
	    digitalWrite(PinRec, LOW);

	  if (Psx.digital_buttons & (psxL1))
            digitalWrite(PinMode, HIGH);
	  else
	    digitalWrite(PinMode, LOW);

          //we need to handle our interface parameter setup here....
          if(Psx.digital_buttons & psxSlct)
          {
            //update the center offset here 
            if(Psx.digital_buttons & psxUp)
            {
             offset++;
             delay(250);  
            }
            if(Psx.digital_buttons & psxDown)
            {
             offset--; 
             delay(250); 
            }
            if(Psx.digital_buttons & psxLeft)
            {
             serv_cal-=1;
             //delay(50);  
            }
            if(Psx.digital_buttons & psxRight)
            {
             serv_cal+=1; 
             //delay(50); 
            }
                        
            //Save all our current settings
            if(Psx.digital_buttons & psxStrt)
            {
              savePresets();
              noInterrupts();
              //Psx.Motorlarge = 0xFF;
              Psx.initcontroller(psxDigital);
              interrupts();
              delay(1000);
              //Psx.Motorlarge = 0x00;
            }
            // flip the joystick for backward operation 
            if(Psx.digital_buttons & psxL2)
            {
              if(flipped)
                flipped = 0;
              else flipped = 1; 
              
              delay(500);
            }
            //do a complete system restart
            if(Psx.digital_buttons & psxJoyL)
              asm volatile ("  jmp 0");

          }
            // do all our position memory settings here
          //
          switch(Psx.digital_buttons & 0xF200)
          {
            //save our preset to temp memory (hold down R2 and press any one of the following 
           case 0x8200:
             presets[0][0] = ATmStepper::getpos1x();
             presets[0][1] = ATmStepper::getpos1y();
            break; 
           case 0x4200: 
             presets[1][0] = ATmStepper::getpos1x();
             presets[1][1] = ATmStepper::getpos1y();          
            break; 
           case 0x2200: 
             presets[2][0] = ATmStepper::getpos1x();
             presets[2][1] = ATmStepper::getpos1y();           
            break;
           case 0x1200: 
             presets[3][0] = ATmStepper::getpos1x();
             presets[3][1] = ATmStepper::getpos1y();           
            break; 
            //recall preset from temp memory 
           case 0x8000: 
             ATmStepper::setpos(presets[0][0], presets[0][1]);
           break;
           case 0x4000: 
             ATmStepper::setpos(presets[1][0], presets[1][1]);
            break; 
           case 0x2000: 
             ATmStepper::setpos(presets[2][0], presets[2][1]);
            break; 
           case 0x1000: 
             ATmStepper::setpos(presets[3][0], presets[3][1]);
            break; 
           default: 
            break;
          }
            
          
  }
  else if (0x40 == (Psx.Controller_mode & 0xF0)) // then we are in digital mode... we should switch to analog
  {
    noInterrupts();
    Psx.initcontroller(psxAnalog);
    interrupts();
    delay(100);
  }
  //Debugging information.... 
  Serial.print("\n");                                            // the button data
  Serial.print(Psx.Controller_mode, HEX);     // prints value as string in hexadecimal (base 16) 
  Serial.print(','); 
  Serial.print(Psx.digital_buttons, HEX);     // prints value as string in hexadecimal (base 16)  
  Serial.print(','); 
  Serial.print(Psx.Right_x, HEX);     // prints value as string in hexadecimal (base 16)    
  Serial.print(Psx.Right_y, HEX);     // prints value as string in hexadecimal (base 16)    
  Serial.print(Psx.Left_x, HEX);     // prints value as string in hexadecimal (base 16)    
  Serial.print(Psx.Left_y, HEX);     // prints value as string in hexadecimal (base 16)      
  Serial.print(','); 
  Serial.print(psxtovelocity(Psx.Left_x), DEC);
  Serial.print(',');

  Serial.print(ATmStepper::getpos1x(), DEC);
  Serial.print(','); 
  Serial.print(ATmStepper::getpos1y(), DEC);
    Serial.print(','); 
  Serial.print(serv_cal, DEC);
  //Serial.print(','); 
  //Serial.print(servPosition, DEC);  
}
