#include "motordriver_4wd.h"
#include "seeed_pwm.h"

#define DIODE_PIN A2

void setup()
{
  digitalWrite(DIODE_PIN, INPUT_PULLUP);
  MOTOR.init();
  MOTOR.setSpeedDir1(16, DIRF);
  MOTOR.setSpeedDir2(8, DIRR);
  state = RIGHT;
  delay(100);
  
  Serial.begin(9600);
}

void loop()
{
  int val = analogRead(DIODE_PIN);

  if (val > 100)
  {
    if (state == LEFT)
    {
      //Set Right
      MOTOR.setSpeedDir1(SPEED_FAST, DIRF);
      //Set Left
      MOTOR.setSpeedDir2(SPEED_SLOW, DIRR);
      state = RIGHT;
      delay(1000);
    }
    else
    if (state == RIGHT)
    {
      //Set Right
      MOTOR.setSpeedDir1(SPEED_SLOW, DIRF);
      //Set Left
      MOTOR.setSpeedDir2(SPEED_FAST, DIRR);
      state = LEFT;
      delay(1000);      
    }
  }

}
