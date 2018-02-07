//The subsytems - these were originally in separate files, but have been compiled here for convenience
// Nicolas Fredrickson 

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Servo Functions and Constants
// Specific to my configuration and servo (HS-422)
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Grove Ultrasonic Functions and Constants
// Specific to my configuration
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#define pingPin A0

#define RANGER_LEFT     ZERO_PI
#define RANGER_FORWARD  PI_O2
#define RANGER_RIGHT    ONE_PI

/*
  ----------Ultrasonic Control Functions ----------------
*/
int pingCM(){
  long duration;

  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);
  delay(100);

  #ifdef DEBUG
  Serial.print("Distance Read: ");
  Serial.print(duration);
  Serial.print(" ---> ");
  Serial.print( (duration / 29) / 2 );
  Serial.println(" cm");
  #endif
  return (duration / 29) / 2;
}

int pingMM(){
  return pingCM() * 10;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Motor Functions and Constants
// Specific to my wheel configuration
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#define TICK_PIN_LEFT 1
#define TICK_PIN_RIGHT 0

#define TICK_PER_ROT    72.0

#define WHEEL_RADIUS    42.5
#define WHEEL_DISTANCE  158.0

#define DISTANCE_PER_TICK   (PI * WHEEL_RADIUS * 2) / TICK_PER_ROT

//The extra speed for our left-side wheels
//If veering left, increase
//If veering right, decrease
#define DIFFERENTIAL_PERCENTAGE   0.0505

//Tick Counts (For Odometry)
volatile int tickCountLeft;
volatile int tickCountRight;

volatile int leftDirection;
volatile int rightDirection;

//Initializer Function
void moveInit(){
  MOTOR.init(); //Initll pin

  pinMode(TICK_PIN_LEFT, INPUT);
  pinMode(TICK_PIN_RIGHT, INPUT);

  tickCountLeft = 0;
  tickCountRight = 0;

  attachInterrupt(TICK_PIN_LEFT, tickLeft, CHANGE);
  attachInterrupt(TICK_PIN_RIGHT, tickRight, CHANGE);
}

/*
  -------------- Tick Counter Functions ----------------
*/
void updateDirection(){
    switch(moveState){
      default:
      case FORWARD:
        leftDirection = 1;
        rightDirection = 1;
        break;
      case TURN_RIGHT:
        leftDirection = 1;
        rightDirection = -1;
        break;
      case TURN_LEFT:
        leftDirection = -1;
        rightDirection = 1;
        break;
      case BACKWARD:
        leftDirection = -1;
        rightDirection = -1;
        break;
    }
}

void tickLeft(){
  tickCountLeft += leftDirection; 
}

void tickRight(){
  tickCountRight += rightDirection;
}
//.375 cm
/*
  -------------- Motor Control Functions ----------------
*/
inline void turnInPlaceLeft(int speeed)
{
  //set right
  MOTOR.setSpeedDir1(speeed, DIRF);
  //set left
  MOTOR.setSpeedDir2(speeed, DIRF);
}

inline void turnInPlaceRight(int speeed)
{
  //set right
  MOTOR.setSpeedDir1(speeed, DIRR);
  //set left
  MOTOR.setSpeedDir2(speeed, DIRR);
}

inline void backward(int speeed)
{
  //set right
  MOTOR.setSpeedDir1(speeed, DIRR);
  //set left
  MOTOR.setSpeedDir2(speeed + DIFFERENTIAL_PERCENTAGE * speeed, DIRF);
}

inline void forward(int speeed)
{
  //set right
  MOTOR.setSpeedDir1(speeed, DIRF);
  //set left
  MOTOR.setSpeedDir2(speeed + DIFFERENTIAL_PERCENTAGE * speeed, DIRR);
}

inline void stopMotion()
{
  //Stop right
  MOTOR.setStop1();
  //Stop left
  MOTOR.setStop2();
}

inline void stopLeft()
{
  //Stop left
  MOTOR.setStop2();
}

inline void stopRight()
{
  //Stop right
  MOTOR.setStop1();
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Button Functions and Constants
// Specific to my configuration
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#define buttonPin 11

//Initializer Function
void buttonInit(){
  pinMode(buttonPin, INPUT);
}

/*
  -------------- Button Functions ----------------
*/

inline void checkButtonState(){
  if( digitalRead(buttonPin) )
    moveState = STOPPED;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// RGB Light Functions and Constants
// Specific to my configuration
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#include <ChainableLED.h>

#define NUM_LEDS    1

#define PIN_CLOCK   SCL
#define PIN_DATA    SDA

#define DEFAULT_SATURATION  1.0
#define DEFAULT_BRIGHTNESS  0.15

#define COLOR_HUE_ONE   0.0
#define COLOR_HUE_TWO   0.25
#define COLOR_HUE_THREE 0.5
#define COLOR_HUE_FOUR  0.75

//defines the pin used on arduino.
ChainableLED light(PIN_CLOCK, PIN_DATA, NUM_LEDS);

void inline lightInit(){
  light.init();
}

/*
  -------------- Light Functions ----------------
*/
inline void radianToColor(float radianAngle){
  light.setColorHSB(0, fmod(radianAngle, TWO_PI) / TWO_PI, DEFAULT_SATURATION, DEFAULT_BRIGHTNESS);
}

inline void degreeToColor(int degree){
  light.setColorHSB(0, (degree % 360) / 360, DEFAULT_SATURATION, DEFAULT_BRIGHTNESS);
}

inline void radianToSat(float radianAngle, double color){
  light.setColorHSB(0, color, fmod(radianAngle, TWO_PI) / TWO_PI ,DEFAULT_BRIGHTNESS );
}

inline void radianToBright(float radianAngle, double color){
  light.setColorHSB(0, color, DEFAULT_SATURATION, fmod(radianAngle, TWO_PI) / TWO_PI );
}

inline void specificColor(double color){
  light.setColorHSB(0, color, DEFAULT_SATURATION, DEFAULT_BRIGHTNESS );
}

inline void rgbColor(int red, int green, int blue){
  light.setColorRGB(0, red, green, blue);
}

inline void lightOff(){
  light.setColorHSB(0, 0, 0, 0);
}

inline void lightCycle(int delayTime){
  for (int i = 0; i < 10; i++)
    {
      rgbColor( 100, 0, 0 );  // red
      delay(delayTime);
      rgbColor( 0, 100, 0 );  // green
      delay(delayTime);
      rgbColor( 0, 0, 100 );  // blue
      delay(delayTime);
    }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Diode (Light Detector) Functions and Constants
// Specific to my configuration
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#define PIN_OUTPUT A2

enum {LEFT, RIGHT} lightState;

void diodeInit(){
  digitalWrite( PIN_OUTPUT, INPUT_PULLUP);
}

int getDiodeVal(){
  return analogRead( PIN_OUTPUT );
}
