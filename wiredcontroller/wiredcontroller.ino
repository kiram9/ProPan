int buttonVal = 0; 
void setup(){
  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);
  
  
  Serial.begin(4800); 
  
  
}


void loop(){
  //read 4 analog values and switch states
  buttonVal = 0;
  Serial.print("u "); 
  Serial.print(map(analogRead(A0),0,1023,0,255)); 
  Serial.print(" "); 
  Serial.print(map(analogRead(A1),0,1023,0,255)); 
  Serial.print(" "); 
  Serial.print(map(analogRead(A2),0,1023,0,255)); 
  Serial.print(" "); 
  Serial.print(map(analogRead(A3),0,1023,0,255)); 
  Serial.print(" "); 
  buttonVal |= digitalRead(10) ? 0:1; 
  buttonVal |= digitalRead(11) ? 0:2; 
  buttonVal |= digitalRead(12) ? 0:4; 
  buttonVal |= digitalRead(13) ? 0:8; 
  Serial.print(buttonVal); 
  Serial.print("\r\n");
  delay(32);
}

