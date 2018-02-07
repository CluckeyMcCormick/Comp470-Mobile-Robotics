// Calibration Functions
// For ensuring that everything is working just right
// Nicolas Fredrickson 

void inline initialTest(){
  //defaultPan();
  basicColorCycle();
}

/*
  -------------- Calibration Functions ----------------
*/

void inline checkTicks(){
  Serial.print(tickCountLeft);
  Serial.print(" : ");
  Serial.println(tickCountRight);
}

void inline tickStopper(){
    if(tickCountLeft > TICK_PER_ROT ) 
      stopLeft();
    if(tickCountRight > TICK_PER_ROT * 2)
      stopRight();
}

//Tests the servo and the ultrasonic ranger
//Pans from 0 to 180 and tests each angle, then aims at the longest distance.
void inline farthest()
{
  int bestAngle = -1;
  int bestPing = -1;

  int ping = 0;

  for (int i = 0; i <= END_ANGLE; i += 10) {
    //Move to angle
    anglePanTo(i);
    //Ping
    ping = pingCM();
    delay(1000);

    //If ping is greater than bestPing
    if (ping > bestPing) {
      //Set the values appropriately
      bestAngle = i;
      bestPing = ping;
    }
  }

  anglePanTo(bestAngle);
}

//Pans to the middle, then the end, then the beginning
void inline defaultPan() {

  Serial.println("To Default Position");

  anglePanTo( (START_ANGLE + END_ANGLE) / 2 );
  delay(500);

  anglePanTo(END_ANGLE);
  delay(500);

  anglePanTo(START_ANGLE);
  delay(500);
}

void inline colorLoop(){

  float hue = 0;
  boolean up = true;

  for(int i = 0; i < 80; i++){
    light.setColorHSB(0, hue, 1.0, 0.5);
    Serial.println(hue);
    delay(100);
  
    if (up)
      hue+= 0.025;
    else
      hue-= 0.025;
  
    if (hue>=1.0 && up)
      up = false;
    else if (hue<=0.0 && !up)
      up = true;
  }
}

void basicColorCycle(){
  light.setColorRGB(0, 128, 0, 0);
  delay(100);
  light.setColorRGB(0, 128, 128, 0);
  delay(100);
  light.setColorRGB(0, 0, 128, 0);
  delay(100);
  light.setColorRGB(0, 0, 128, 128);
  delay(100);
  light.setColorRGB(0, 0, 0, 128);
  delay(100);
  light.setColorRGB(0, 128, 0, 128);
  delay(100);
  light.setColorRGB(0, 128, 128, 128);
  delay(100);
}

