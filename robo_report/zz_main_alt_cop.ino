#include "motordriver_4wd.h"
#include "structs.h"
#include <seeed_pwm.h>
#include <StackArray.h>

//#define DEBUG 1
//#define DEBUG_PING_CHECK 1
//#define DEBUG_PATH 1
//#define DELAY_START 1

enum {FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT, STOPPED} moveState;
enum {WANDER, WANDEROUT, GOHOME, INCLINE, DONE} machineState;

//Calibration function prototypes
void inline checkTicks();
void inline defaultPan();
void inline tickStopper();
void inline initialTest();
void inline lightInit();
void inline colorLoop();
void inline farthest();

void setup()
{
  //Individually initialize our components
  servoInit();
  moveInit();
  buttonInit();
  lightInit();
  diodeInit();
  worldNavInit();
  
  initialTest();

  clavInit();

  #ifdef DELAY_START
  lightCycle(250);
  #endif
  
  //Initialize the serial communications:
  Serial.begin(9600);
}

void loop()
{
  //forward(20);
  clavBrain();
  //farthest();
  //Serial.println( getDiodeVal() );
}

