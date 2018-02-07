// Motor Functions and Constants
// Specific to my wheel configuration
// Nicolas Fredrickson 

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

