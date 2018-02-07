#include <motordriver_4wd.h>
#include <seeed_pwm.h>
#include <ChainableLED.h>

//-------- PIE constants

const double PIE    = 3.14159265;
const double PIE_O2 = PIE/2.0;
const double PIE2 = PIE*2.0;

//-------- LED crap
#define PIN_CLOCK   SCL
#define PIN_DATA    SDA
#define DEFAULT_SATURATION  1.0
#define DEFAULT_BRIGHTNESS  0.15

#define NUM_LEDS  1
ChainableLED leds(PIN_CLOCK, PIN_DATA, NUM_LEDS);

//-------- motor control 
#define TURN_SPEED 40

void TurnLeft90();
void TurnRight90();
void Straight( int speed, int dirn );


//-------- bumper
const int buttonPin = 11;
int button_state = 0;

//-------- dead reckoning 

// ticks per rotation
#define TPR 72
 
// robot measurements (mm)
#define RW    42.5  // radius wheel
#define D     158.0

//The amount of ticks to turn 90 degrees, per encoder (outside assumed)
#define NINETY_DEGREE_AT_40_LEFT    63  //Left ticks to turn right
#define NINETY_DEGREE_AT_40_RIGHT   55  //Right ticks to turn left

// robot config variables
double x = 50.0, y = 50.0, dx = 0.0, dy = 0.0;
double theta =  PI/2.0;

// encoder variables
volatile long left_encoder_count = 0, right_encoder_count = 0;   
int left_dirn = 1, right_dirn = 1;


//-------- robot state 

enum {FWD, REV} state;

//-------- model of environment 

double LEFT = 0.0;
double RIGHT = 1500.0;
double BOTTOM = 0.0;
double TOP = 1500.0;

//======================================================================================
// Custom Motor
//======================================================================================
inline void turnInPlaceLeft(int speeed)
{
  MOTOR.setSpeedDir(speeed, DIRF);
}

inline void turnInPlaceRight(int speeed)
{
  MOTOR.setSpeedDir(speeed, DIRR);
}

inline void backward(int speeed)
{
  MOTOR.setSpeedDir1(speeed, DIRR);
  MOTOR.setSpeedDir2(speeed, DIRF);
}

inline void forward(int speeed)
{
  MOTOR.setSpeedDir1(speeed, DIRF);
  MOTOR.setSpeedDir2(speeed, DIRR);
}

inline void stopMotion()
{
  MOTOR.setStop1();
  MOTOR.setStop2();
}

inline void stopLeft()
{
  MOTOR.setStop2();
}

inline void stopRight()
{
  MOTOR.setStop1();
}

//======================================================================================
// Custom Light
//======================================================================================
inline void radianToColor(float radianAngle){
  leds.setColorHSB(0, fmod(radianAngle, TWO_PI) / TWO_PI, DEFAULT_SATURATION, DEFAULT_BRIGHTNESS);
}

inline void degreeToColor(int degree){
  leds.setColorHSB(0, (degree % 360) / 360, DEFAULT_SATURATION, DEFAULT_BRIGHTNESS);
}

//======================================================================================
// setup
//======================================================================================
void setup()
{
    leds.init();
    MOTOR.init();

    attachInterrupt(0, LeftEncoder, CHANGE);
    attachInterrupt(1, RightEncoder, CHANGE);
        
    // go straight
    Straight( 10, 1 ); 
    leds.setColorRGB(0, 0, 100, 0);  // green
    state = FWD;    
       
    //Serial.begin(9600);
}

//======================================================================================
// Loop
//======================================================================================
void loop()
{
  delay(100);
  
  //---- update robot config (x,y,theta)
  dx = PIE * RW * cos(theta) * ((double)(left_encoder_count + right_encoder_count) / TPR);
  x = x + dx;
  
  dy = PIE * RW * sin(theta) * ((double)(left_encoder_count + right_encoder_count) / TPR);
  y = y + dy;
  
  right_encoder_count = left_encoder_count = 0;
  
  //---- a simple two-state behavior to stay in the box and detect bumps

  if( digitalRead(buttonPin) == 1) {button_state = 1;}; 

  // check state

  if ( (state == FWD) && button_state == 1 )
  {
    //---- stop
    Straight( 0, 0 );    
    delay(100);

    //---- back up
    leds.setColorRGB(0, 100, 0, 0);  // red
    Straight( 10, -1 );    
    delay(500);

    button_state = 0;

    //---- update state
    state = REV;   
  } 
  else if ((state == FWD) && (x >= RIGHT || x <= LEFT || y >= TOP || y <= BOTTOM))
  {
    //---- stop
    Straight( 0, 0 );    
    delay(100);

    //---- back up
    leds.setColorRGB(0, 100, 0, 0);  // red
    Straight( 10, -1 );    
    delay(500);

    //---- update state
    state = REV;    
  }
  else
  if ((state == REV) && (x < RIGHT && x > LEFT && y < TOP && y > BOTTOM))
  {
    //---- stop
    Straight( 0, 0 );    

    //---- turn right 90
    leds.setColorRGB(0, 0, 0, 100);
    TurnRight90();
    //---- update robot config (theta)
    theta  = fmod(theta - PIE_O2, PIE2);
    delay(100);

    //---- go straight
    leds.setColorRGB(0, 0, 100, 0);  // green
    Straight( 10, 1 );   

    //---- update state
    state = FWD;    
  }

}


//======================================================================================
// TurnLeft90
//======================================================================================
void
TurnLeft90()
{
    right_encoder_count = left_encoder_count = 0;
    
    left_dirn = -1; right_dirn = 1;
    turnInPlaceLeft(TURN_SPEED);
    while (right_encoder_count < NINETY_DEGREE_AT_40_RIGHT)
    {
      delayMicroseconds(1);
    }

    stopMotion();
}

//======================================================================================
// TurnRight90
// dirn is 1 for right, -1 for left
//======================================================================================
void
TurnRight90()
{
    right_encoder_count = left_encoder_count = 0;
    
    left_dirn = 1; right_dirn = -1;
    turnInPlaceRight(TURN_SPEED);
    while (left_encoder_count < NINETY_DEGREE_AT_40_LEFT)
    {
      delayMicroseconds(1);
    }

    stopMotion();
}

//======================================================================================
// Straight
// dirn is 1 for fwd, -1 for bwd
//======================================================================================
void
Straight( int speed, int dirn )
{
    //---- setup encoder variables
    left_dirn = dirn; right_dirn = dirn;
    
    if (speed == 0)       //-- stop
    {
      stopMotion();   return;
    }
    else if (dirn == 1)   //-- fwd
    {
      forward(speed);
    }
    else                  //-- bwd
    {
      backward(speed);
    }
}

//======================================================================================
// Interrupt Service Routines for encoders
//======================================================================================
void LeftEncoder()
{
  left_encoder_count = left_encoder_count + left_dirn;
}

void RightEncoder()
{
  right_encoder_count = right_encoder_count + right_dirn;
}
