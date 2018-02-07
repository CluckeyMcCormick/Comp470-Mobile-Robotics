#include "motordriver_4wd.h"
#include <seeed_pwm.h>

long left_encoder_count = 0;
long right_encoder_count = 0;
int left_dir = 1;
int right_dir = 1;

void setup() {
  // put your setup code here, to run once:
  MOTOR.init();
  //
  attachInterrupt(0, RightEncoder, CHANGE);
  attachInterrupt(1, LeftEncoder, CHANGE);
  //

  MOTOR.setSpeedDir1(10, DIRF);
  MOTOR.setSpeedDir2(10, DIRR);
  
   Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("left_encoder_count: "); Serial.print(left_encoder_count);
  Serial.print(" right_encoder_count: "); Serial.println(right_encoder_count);
  right_encoder_count = left_encoder_count = 0;

  delay(1000);
}

void LeftEncoder()
{
  left_encoder_count = left_encoder_count + left_dir;
}

void RightEncoder()
{
  right_encoder_count = right_encoder_count + right_dir;
}



