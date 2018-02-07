#include <motordriver_4wd.h>
#include <seeed_pwm.h>
#include <ChainableLED.h>

//-------- PIE constants

const double PIE    = 3.14159265;
const double PIE_O2 = PIE/2.0;
const double PIE2   = PIE*2.0;

//-------- LED crap
#define PIN_CLOCK   SCL
#define PIN_DATA    SDA
#define NUM_LEDS  1

#define DEFAULT_SATURATION  1.0
#define DEFAULT_BRIGHTNESS  0.15

ChainableLED leds(SCL, SDA, NUM_LEDS);

//-------- motor control 

void Turn( int dirn );
void Straight( int speed, int dirn );

//-------- dead reckoning 

// ticks per rotation
#define TPR 72
 
// robot measurements (mm)
#define RW    42.5  // radius wheel
#define D     158.0

//The amount of ticks to turn 90 degrees, per encoder (outside assumed)
#define NINETY_DEGREE_AT_40_LEFT    63  //Left ticks to turn right
#define NINETY_DEGREE_AT_40_RIGHT   55  //Right ticks to turn left

// robot configuration variables
double x = 100.0, y = 100.0, dx = 0.0, dy = 0.0, theta =  PI/2.0, dtheta = 0.0;

// encoder variables
volatile long left_encoder_count = 0, right_encoder_count = 0;   
volatile int left_dirn = 1, right_dirn = 1;
int encoder_diff = 0;

//-------- robot state 

enum {FWD, REV} state;

//-------- model of environment 

double LEFT = 0.0;
double RIGHT = 1500.0;
double BOTTOM = 0.0;
double TOP = 1500.0;

float angleAdditive = 0;

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

    attachInterrupt(1, LeftEncoder, CHANGE);
    attachInterrupt(0, RightEncoder, CHANGE);
        
    // go straight
    Straight( 10, 1 ); 
       
    Serial.begin(9600);
}

//======================================================================================
// Loop
//======================================================================================
void loop()
{
  //Serial.print(x); Serial.print(" :x - y: "); Serial.println(y);
  delay(100);
  
  //---- update robot config (x,y,theta)
  dx = PIE * RW * cos(theta) * ((double)(left_encoder_count + right_encoder_count) / TPR);
  x = x + dx;
  
  dy = PIE * RW * sin(theta) * ((double)(left_encoder_count + right_encoder_count) / TPR);
  y = y + dy;
  
  encoder_diff = left_encoder_count - right_encoder_count;

  //---- check for boundaries
  if ((state == FWD) && (x >= RIGHT || x <= LEFT || y >= TOP || y <= BOTTOM))
  {
    //---- stop
    Straight( 0, 0 );    

    //---- back up
    Straight( 10, -1 );    
    delay(500);
  }
  else
  if ((state == REV) && (x < RIGHT && x > LEFT && y < TOP && y > BOTTOM))
  {
    //---- stop
    Straight( 0, 0 );    

    //---- turn right 90
    Turn(1);

    //---- go straight
    Straight( 10, 1 );    
  }
  else
  {
    right_encoder_count = left_encoder_count = 0;
  }
  
  radianToColor(theta + angleAdditive);
}

//======================================================================================
// Turn
// dirn is 1 for right, -1 for left
//======================================================================================
void
Turn( int dirn )
{
    //---- turn 90    
    leds.setColorRGB(0, 0, 0, 100);
    right_encoder_count = left_encoder_count = 0;
    
    if ( dirn == 1)
    {
      left_dirn = 1; right_dirn = -1;
      //MOTOR.setSpeedDir1(40, DIRF); MOTOR.setSpeedDir2(40, DIRF);
      turnInPlaceRight(40);
      while (left_encoder_count < NINETY_DEGREE_AT_40_LEFT)
      {
        delayMicroseconds(1);
        //Serial.print('\t'); 
        //Serial.print(left_encoder_count); Serial.println(" :LEFT");// - RIGHT: "); Serial.println(right_encoder_count);
        //radianToColor( theta - PIE_O2 + (left_encoder_count * .1 ) );
      }
      theta  = fmod(theta - PIE_O2, PIE2);
    }
    else
    {
      left_dirn = -1; right_dirn = 1;
      //MOTOR.setSpeedDir1(40, DIRR); MOTOR.setSpeedDir2(40, DIRR);
      turnInPlaceLeft(40);
      while (right_encoder_count < NINETY_DEGREE_AT_40_RIGHT)
      {
        delayMicroseconds(1);
        //Serial.print('\t'); 
        //Serial.print(right_encoder_count); Serial.print(" :RIGHT - LEFT: "); Serial.println(left_encoder_count);
        //radianToColor( theta + PIE_O2 + (right_encoder_count * .1 ) );    
      }
      theta  = fmod(theta + PIE_O2, PIE2);
    }

    stopMotion();
    
    delay(100);
    
}

//======================================================================================
// Straight
// dirn is 1 for fwd, -1 for bwd
//======================================================================================
void
Straight( int speed, int dirn )
{
    if (speed == 0) 
    {
      stopMotion();
      delay(100);
      return;
    }
  
    //---- setup encoder variables
    left_dirn = dirn; right_dirn = dirn;
    right_encoder_count = left_encoder_count = 0;
            
    if (dirn == 1)
    {
      //MOTOR.setSpeedDir1(speed+0.2*speed-encoder_diff, DIRF); MOTOR.setSpeedDir2(speed+encoder_diff, DIRR); 
      forward(speed);

      angleAdditive = 0;

      state = FWD;
      //leds.setColorRGB(0, 0, 100, 0);  // green
    }
    else
    {
      //MOTOR.setSpeedDir1(speed+0.2*speed-encoder_diff, DIRR); MOTOR.setSpeedDir2(speed+encoder_diff, DIRF);
      backward(speed);

      angleAdditive = -PI;
      state = REV;
      //leds.setColorRGB(0, 100, 0, 0);  // red
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
