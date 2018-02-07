
#define COLOR_HUE_FORWARD   0.0
#define COLOR_HUE_LEFT   0.25
#define COLOR_HUE_BACKWARD 0.5
#define COLOR_HUE_RIGHT  0.75

#define CLAV_BRAIN_TURN_SPEED_LEFT  40
#define NINETY_DEGREE_LEFT  69//67  //Right ticks to turn left

#define CLAV_BRAIN_TURN_SPEED_RIGHT 40
#define NINETY_DEGREE_RIGHT 69  //Left ticks to turn right //66 //70

#define PING_TARGET_CM_STOP   60 //We can see two boxes away!
#define PING_TARGET_CM_MOVE   24
//#define PING_TARGET_MM   500

#define INVESTIGATE_ITERATES    2

#define WANDER_TARGET_COL  4
#define WANDER_TARGET_ROW  4

#define GO_HOME_TARGET_COL  0
#define GO_HOME_TARGET_ROW  0

#define INCLINE_TARGET_COL  -5
#define INCLINE_TARGET_ROW  0

#define INCLINE_TARGET_THETA  ONE_PI

#define INCLINE_SPEED_FAST 16
#define INCLINE_SPEED_SLOW 8
#define COLLISION_SPEED   5
#define WANDER_SPEED      10
#define KNOWN_SPEED       15

int targetCol, targetRow;
boolean parity;

//======================================================================================
// Initializer
//======================================================================================
inline void clavInit(){
  tickCountLeft = tickCountRight = 0;
  moveState = STOPPED;
  updateDirection();

  specificColor(COLOR_HUE_FORWARD);

  dTheta = 0;
  dX = 0;
  dY = 0;

  x = EXCESS / 2; //0;
  y = EXCESS / 2; //0;//200;//50.0;//80.0;
  thetaState = PI_O2;
  theta = radianToValue(thetaState);
  addShift(thetaState);

  targetCol = WANDER_TARGET_COL;
  targetRow = WANDER_TARGET_ROW;

  radianPanTo( RANGER_FORWARD );
  delay(50);
  parity = false;
  //anglePanTo(90);
  //Straight( 10, 1 );
}

//======================================================================================
// Loop
//======================================================================================
inline void clavBrain()
{
  
  //if(moveState == FORWARD)
    //pingCheck();

  //delay(100);
  
  //---- update robot config (x,y,theta)
  dX = PI * WHEEL_RADIUS * cos(theta) * ((double)(tickCountLeft + tickCountRight) / TICK_PER_ROT);
  xNoSkew = xNoSkew + dX;
  x = x + dX;
  
  dY = PI * WHEEL_RADIUS * sin(theta) * ((double)(tickCountLeft + tickCountRight) / TICK_PER_ROT);
  yNoSkew = yNoSkew + dY;
  y = y + dY;
  
  
  tickCountLeft = tickCountRight = 0; 

  markCurrentPass();
  
  #ifdef DEBUG
  Serial.print(x); Serial.print(" :x || y: "); Serial.println(y); Serial.print("\t"); 
  Serial.print(valueToSectorMM( x )); Serial.print(" :col || row: "); Serial.println(valueToSectorMM( y ));
  #endif

  //Change our light
  if(parity)
    radianToColor( radianToValue( moveEffectTheta() ) );
  else
    radianToColor( theta );

  parity = !parity;
  
  if(machineState == WANDER || machineState == WANDEROUT){
    clavGoPlace(true);
    //---- check if we're completely in the
    if ( posInTarget(targetCol, targetRow) )
    {
      ClavStop();
      
      #ifdef DEBUG
      Serial.println("======================");
      Serial.println("\t Wander Target Reached!");
      Serial.println("======================");
      #endif

      //---- update state
      radianPanTo( RANGER_FORWARD );
      delay(100);
      
      machineState = GOHOME; 
      targetCol = GO_HOME_TARGET_COL;
      targetRow = GO_HOME_TARGET_ROW;
    } 
  }
  else if (machineState == GOHOME)
  {
    //---- use map to go home
    clavGoPlace(false);
    
    //---- check if we're completely in the
    if ( posInTarget(targetCol, targetRow) )
    {
      ClavStop();

      #ifdef DEBUG
      Serial.println("======================");
      Serial.println("\t Go Home - Target Reached!");
      Serial.println("======================");
      #endif
      
      machineState = INCLINE; 
      targetCol = INCLINE_TARGET_COL;
      targetRow = INCLINE_TARGET_ROW; 
      
      turnToTheta(ONE_PI);
      
      lightCycle(50);
    } 
  }
  else if (machineState == INCLINE) {
    if ( getDiodeVal() > 100) {
      if (lightState == LEFT) {
        //Set Right
        MOTOR.setSpeedDir1(INCLINE_SPEED_FAST, DIRF);
        //Set Left
        MOTOR.setSpeedDir2(INCLINE_SPEED_SLOW, DIRR);
        lightState = RIGHT;
        delay(100);
      }
      else if (lightState == RIGHT){
        //Set Right
        MOTOR.setSpeedDir1(INCLINE_SPEED_SLOW, DIRF);
        //Set Left
        MOTOR.setSpeedDir2(INCLINE_SPEED_FAST, DIRR);
        lightState = LEFT;
        delay(100);      
      }
    }
    
    //---- check if we're completely in the
    if ( x <= -1550 )
    {
      ClavStop();

      #ifdef DEBUG
      Serial.println("======================");
      Serial.println("\t Incline - Target Reached!");
      Serial.println("======================");
      #endif
      
      machineState = DONE; 
    }
  }
  else if (machineState == DONE){
    lightCycle(100);
  }
}

//~~~~~~~~~~~~~
// Sub-brain: Wander (for wandering, and the like)
//~~~~~~~~~~~~~
void clavGoPlace(boolean investigate){

  //If we've stopped
  if( moveState == STOPPED ){
    int speedWeDo;
    //Investigate the surrounding area
    if(investigate){
      investigatePing();
      speedWeDo = WANDER_SPEED;
    } 
    else
      speedWeDo = KNOWN_SPEED;
      
    #ifdef DEBUG
    printPassGrid();
    delay(100);
    printStatGrid();
    delay(100);
    #endif
      
    //Formulate a plan
    pathFindFromCurrent(targetCol, targetRow);

    tickCountLeft = tickCountRight = 0;
    
    if(investigate){
      radianPanTo( RANGER_FORWARD ); delay(50);
    }

    #ifdef DEBUG
    Serial.println( getStartAction() );
    #endif

    //Follow the plan
    switch( getStartAction() ){
      default: 
      case ACTION_STOP:
      case ACTION_NONE:
        break;
        
      case ACTION_FORWARD: 
        ClavForward(speedWeDo);break;
        
      case ACTION_REVERSE: 
        ClavReverse(speedWeDo); break;
        
      case ACTION_LEFT_FORWARD: 
        TurnLeft90(); ClavForward(speedWeDo); break;
      
      case ACTION_RIGHT_FORWARD: 
        TurnRight90(); ClavForward(speedWeDo); break;
    }
  }
  //If we've reached our subtarget
  else if( posInSubTarget() ){
    //Stop!
    ClavStop();
    #ifdef DEBUG
    Serial.println("======================");
    Serial.println("\t Subtarget Reached!");
    Serial.println("======================");
    #endif   
  }
  // If we're moving and we're suddenly attacked by a wild box
  else if( moveState == FORWARD && digitalRead(buttonPin) )
  {
    //---- stop
    ClavStop();
    //---- mark that we had a collision, adjust our sights
    blockCollision();

    tickCountLeft = tickCountRight = 0;

    ClavReverse(COLLISION_SPEED);
    
    #ifdef DEBUG
    Serial.println("======================");
    Serial.println("\t Button Pressed!");
    Serial.println("======================");
    #endif   
  }
  //otherwise, if we're moving forward and we go out of bounds
  else if( (moveState == FORWARD || moveState == BACKWARD) && outOfBoundry() && machineState == WANDER )//(OUT_OF_BOUNDRY) )
  {
    int oldState = moveState;
    //---- stop
    ClavStop();

    //Mark that we've gone out of bounds
    machineState = WANDEROUT;

    #ifdef DEBUG
    Serial.println("======================");
    Serial.println("\tOut of Bounds!");
    Serial.println("======================");
    #endif 

    if(oldState == FORWARD)
      ClavReverse(WANDER_SPEED);
    else if(oldState == BACKWARD)
      ClavForward(WANDER_SPEED);
  }
  else if ( machineState == WANDEROUT && inBoundry() )//(IN_BOUNDRY))
  {
    //---- stop
    ClavStop();
    machineState = WANDER;
  }
  /*
  else if(machineState == WANDER || machineState == WANDEROUT){
    projectionCheck(); 
  }
  */
}

//======================================================================================
// Initiate the forward - this code is duplicated so much that we might as well
//======================================================================================
inline void ClavForward(int inSpeed) {
  //---- update state
  moveState = FORWARD; 
  updateDirection();
  //---- go forward
  forward(inSpeed); 
  //---- R E S T    A E S T H E T I C A L L Y
  delay(100);
}

//======================================================================================
// Initiate the reverse - this code is duplicated so much that we might as well
//======================================================================================
inline void ClavReverse(int inSpeed) {
  //---- update state
  moveState = BACKWARD; 
  updateDirection();
  //---- back up
  backward(inSpeed);
  //---- R E S T    A E S T H E T I C A L L Y 
  delay(100);
}

//======================================================================================
// Stop the car - this code is duplicated so much that we might as well
//======================================================================================
inline void ClavStop() {
  //---- stop
  stopMotion();
  //---- R E S T    A E S T H E T I C A L L Y
  delay(100);
  //---- update state
  moveState = STOPPED;
  updateDirection();
}

//======================================================================================
// TurnLeft90
//======================================================================================
void TurnLeft90() {
    //Set up for turning left
    specificColor(COLOR_HUE_LEFT);
    moveState = TURN_LEFT;
    updateDirection();
    tickCountLeft = tickCountRight = 0;

    //begin the turn
    turnInPlaceLeft(CLAV_BRAIN_TURN_SPEED_LEFT);

    //while loop to turn as we please
    while (tickCountRight < NINETY_DEGREE_LEFT)
    {
      delayMicroseconds(1);
    }

    //Stop the turn
    ClavStop();
  
    #ifdef DEBUG
    Serial.println("======================");
    Serial.println("\t Turning Left!");
    Serial.println("======================");
    #endif

    tickCountLeft = tickCountRight = 0;

    removeShift(thetaState);

    //Update our theta (positively)
    incrementTheta();

    addShift(thetaState);

    //R E S T    A E S T H E T I C A L L Y
    delay(100);
}

//======================================================================================
// TurnRight90
// dirn is 1 for right, -1 for left
//======================================================================================
void TurnRight90() {
    //Set up for turning right
    specificColor(COLOR_HUE_RIGHT);
    moveState = TURN_RIGHT; 
    updateDirection();
    tickCountLeft = tickCountRight = 0;

    //begin the turn
    turnInPlaceRight(CLAV_BRAIN_TURN_SPEED_RIGHT);

    //while loop to turn as we please
    while (tickCountLeft < NINETY_DEGREE_RIGHT)
    {
      delayMicroseconds(1);
    }
    
    //Stop the turn
    ClavStop();

    #ifdef DEBUG
    Serial.println("======================");
    Serial.println("\t Turning Right!");
    Serial.println("======================");
    #endif

    tickCountLeft = tickCountRight = 0;

    removeShift(thetaState);

    //Update our theta (negatively)
    decrementTheta();

    addShift(thetaState);

    //R E S T    A E S T H E T I C A L L Y
    delay(100);
}

//======================================================================================
// TurnToTheta
//======================================================================================
void turnToTheta(int target){
  switch( target - thetaState ){
    case -1: TurnRight90();
      break;
    case 1: TurnLeft90();
      break;
    case 2:
    case -2: TurnRight90(); TurnRight90();
      break;
    default:
    case 0:
      break;
  }
}

//Take a look around, see?
void investigatePing(){
  //start left
  int angle = RANGER_LEFT;
  
  //for each angle
  for(int i = 0; i < 3; i++){
    //Pan to that angle
    radianPanTo(angle);
    delay(50);

    //If whatever we're scanning isn't automatically going to be out of bounds
    if( inSectors(valueToSectorMM( x ) + projectColPan(), valueToSectorMM( y ) + projectRowPan()) ){
      //Then ping X times
      for(int j = 0; j < INVESTIGATE_ITERATES; j++){
        pingCheck();
        delay(100);
      } 
    }
    angle = radianLeft(angle);
  }
  evalPanTheta();
}

inline void pingCheck(){ 
  boolean test;
  int dist_cm = pingCM();
  double objX, objY;
  int objRow, objCol;
  
  if( pingMoveCheck(dist_cm) || pingStopCheck(dist_cm) ) {
    objX = (x / 10) + cos( evalPanTheta() ) * dist_cm;
    objY = (y / 10) + sin( evalPanTheta() ) * dist_cm;

    objCol = valueToSectorCM( objX );
    objRow = valueToSectorCM( objY );

    if( objCol == valueToSectorMM( x ) && objRow == valueToSectorMM( y ) ){
        objCol += projectColPan();
        objRow += projectRowPan();
    }
    
    if( inSectors( objCol, objRow ) )  
    { 
      if( getSectorStat( objCol, objRow ) <= 0.0 )
        setSectorStat( objCol, objRow, 0.5);
        
      incrementSectorStat( objCol, objRow, ( 1.0 - getSectorStat( objCol, objRow ) ) / 2.0 );
    }
  }
}

inline boolean pingMoveCheck(int dist_cm){
  return (moveState == FORWARD || moveState == BACKWARD) && dist_cm < PING_TARGET_CM_MOVE;
}

inline boolean pingStopCheck(int dist_cm){
  return moveState == STOPPED && dist_cm < PING_TARGET_CM_STOP;
}
