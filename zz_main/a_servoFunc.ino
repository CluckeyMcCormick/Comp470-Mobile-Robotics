// Servo Functions and Constants
// Specific to my configuration and servo (HS-422)
// Nicolas Fredrickson 

#define SERVO_PULSE_MIN   0.4 * 1000 // This should be your 0 degrees
#define SERVO_PULSE_MAX   2.4 * 1000 // This should be your 180 degrees
#define SERVO_PERIOD      25 * 1000  // Total pulse length, mis-setting can result in erratic behavior!

#define START_ANGLE   0
#define END_ANGLE     180

#define servoPin 13

IncrementalTheta panDir;

void inline defaultPan();

//Initializer Function
void servoInit(){
  pinMode(servoPin, OUTPUT);
}

/*
  --------------- Servo Control Functions ---------------
*/

//Executes a PWM order according to the provided pulse length
void pulsePanTo(float pulseLength) {
  for (int i = 0; i < 200; i++) {
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(pulseLength);
    digitalWrite(servoPin, LOW);
    delayMicroseconds(SERVO_PERIOD - pulseLength);
  }
}

void radianPanTo( int rad ){
  switch(rad){   
    case ZERO_PI: 
      panDir = ZERO_PI; anglePanTo(0);
      break;
    default:
    case PI_O2: 
      panDir = PI_O2; anglePanTo(90);
      break;
    case ONE_PI: 
      panDir = ONE_PI; anglePanTo(180);
      break;
  }
}

//Converts a target angle into a target pulse length, then pans
//Essentially, maps the angle from the angle range (START_ANGLE to END_ANGLE)
//to the pulse range (SERVO_PULSE_MIN to SERVO_PULSE_MAX)
void anglePanTo(int angle) {
  float newVal = angle - START_ANGLE;
  newVal = newVal * (SERVO_PULSE_MAX - SERVO_PULSE_MIN);
  newVal = newVal / (END_ANGLE - START_ANGLE);
  newVal = newVal + SERVO_PULSE_MIN;

  pulsePanTo(newVal);
}
