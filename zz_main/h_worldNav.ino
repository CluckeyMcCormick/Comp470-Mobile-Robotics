//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Worldgrid Constants 
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#define DEFAULT_CERTAINTY 0.0

#define PASSED_UNSURE   '?' // Haven't crossed it, wouldn't know 
#define PASSED_BLOCKED  '#' // Obstacle, space is blocked
#define PASSED_OPENED   '*' // Non-obstacle, space is clear
#define PASSED_MISREAD  'E' // YOU FOOL, WHAT HAVE YOU DONE!?!?!?

#define ROW_COUNT     5
#define COLUMN_COUNT  5

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// World Navigation Constants
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

#define SECTOR_WIDTH_CM  30
#define SECTOR_WIDTH_MM  300.0

#define LEFT_BOUNDRY    0.0
#define RIGHT_BOUNDRY   COLUMN_COUNT * SECTOR_WIDTH_MM
#define BOTTOM_BOUNDRY  0.0
#define TOP_BOUNDRY     ROW_COUNT * SECTOR_WIDTH_MM

#define EXCESS 192//187.5

//======================================================================================
// Variables!
//======================================================================================

//Tracks our statistical certainty about the existence of an obstacle
//Used with our distance ranger
float statGrid[ROW_COUNT][COLUMN_COUNT];

//Records our absolute certainty about the existence of an obstacle
//Used when we pass through a sector.
char passGrid[ROW_COUNT][COLUMN_COUNT];

IncrementalTheta thetaState;

//Odometry Variables
double dTheta;
double dX;
double dY;

double x;
double y;
double theta;

double xNoSkew;
double yNoSkew;

//======================================================================================
// Initialization!
//======================================================================================
inline void worldNavInit(){
  for (int i = 0; i < ROW_COUNT; i++) {
    for (int j = 0; j < COLUMN_COUNT; j++) {
      statGrid[i][j] = DEFAULT_CERTAINTY;
      passGrid[i][j] = PASSED_UNSURE;
    }
  } 
}

inline void addShift( IncrementalTheta inThet ){
  xNoSkew = x;
  yNoSkew = y;
  
  switch(inThet){
    default:
    case ZERO_PI:
      //Undo the slight add, the do the excess add
      x = x + (EXCESS / 2) + 15;
      break;
    case PI_O2:
      y = y + (EXCESS / 2) - 5;
      break;
    case ONE_PI:
      x = x + (EXCESS / 4);
      break;
    case THREE_PI_O2:
      y = y + (EXCESS / 4);
      break;  
  }
}  
   
inline void removeShift( IncrementalTheta inThet ){
  switch(inThet){
    default:
    case ONE_PI:
    case ZERO_PI:
      x = xNoSkew;
      break;
    case PI_O2:
    case THREE_PI_O2:
      y = yNoSkew;
      break;  
  }
} 

//======================================================================================
// Grid Coordinate Resolution & Management - for both grids
//======================================================================================

// Think of the sectors as a number line
// Out input is, essentially, a point on the number line 
inline int valueToSectorCM( double cmVal ){
  return int( cmVal / SECTOR_WIDTH_CM );
}

inline int valueToSectorMM( double mmVal ){
  return int( mmVal / SECTOR_WIDTH_MM );
}

boolean inSectors(int col, int row){
  return (col >= 0 && row >= 0) && (col < COLUMN_COUNT && row < ROW_COUNT); 
}

//======================================================================================
// Statistics Grid Management
//======================================================================================

double getSectorStat( int col, int row ){
  if( inSectors(col, row) )
    return statGrid[ row ][ col ];
  else
    return -1.0;
}

void setSectorStat( int col, int row, double value ){
  if( inSectors(col, row) )
    statGrid[ row ][ col ] = value; 
}

void incrementSectorStat( int col, int row, double value ){
  if( inSectors(col, row) )
    statGrid[ row ][ col ] += value; 
}

void printStatGrid(){
  for(int i = ROW_COUNT - 1; i >= 0; i--){
    for(int j = 0; j < COLUMN_COUNT; j++){
      Serial.print(statGrid[i][j]); Serial.print(' ');
    }
    Serial.println();   
  }
}

//======================================================================================
// Passed Grid Management
//======================================================================================

inline char getSectorPass( int col, int row ){
  if( inSectors(col, row) )
    return passGrid[ row ][ col ];
  else
    return PASSED_MISREAD;
}

inline void setSectorPass( int col, int row, char symbol ){
  if( inSectors(col, row) )
    passGrid[ row ][ col ] = symbol; 
}

inline boolean getSectorBlock( int col, int row ){
  if( inSectors(col, row) )
    return passGrid[ row ][ col ] == PASSED_BLOCKED;
  else
    return false;
}

inline void markCurrentPass(){
  setSectorPass( valueToSectorMM( x ), valueToSectorMM( y ), PASSED_OPENED);
}

inline void markProjectedBlock(){
  setSectorPass( valueToSectorMM( x ) + projectColTheta(), valueToSectorMM( y ) + projectRowTheta(), PASSED_BLOCKED);
}

inline void printPassGrid(){
  for(int i = ROW_COUNT - 1; i >= 0; i--){
    for(int j = 0; j < COLUMN_COUNT; j++){
      Serial.print(passGrid[i][j]); Serial.print(' ');
    }
    Serial.println();   
  }
}

//======================================================================================
// Theta Management
//======================================================================================
void incrementTheta(){
  thetaState = radianLeft(thetaState);
  theta = radianToValue(thetaState);
}

void decrementTheta(){
  thetaState = radianRight(thetaState);
  theta = radianToValue(thetaState);
}

IncrementalTheta getPanTheta(){
  if(panDir == RANGER_LEFT)
    return radianLeft(thetaState);
  else if(panDir == RANGER_RIGHT)
    return radianRight(thetaState);
  else
    return thetaState;
}

double evalPanTheta(){
  return radianToValue( getPanTheta() );
}

//Find our effective theta given our current direction
IncrementalTheta moveEffectTheta(){
  return customEffectTheta(moveState);
}

//Find our effective theta given our current direction
IncrementalTheta customEffectTheta(int inMove){
  if(inMove == BACKWARD)
    return (thetaState + 2) % 4;
  else
    return thetaState;
}

//======================================================================================
// Projections - for finding out what's in front of us.
//======================================================================================

int projectCol(IncrementalTheta inputTheta){
  switch( inputTheta ){
    default:
    case ZERO_PI: return 1;
    case PI_O2: return 0;
    case ONE_PI: return -1;
    case THREE_PI_O2: return 0;
  }
  return 0;
}

int projectRow(IncrementalTheta inputTheta){
  switch( inputTheta ){
    default:
    case ZERO_PI: return 0;
    case PI_O2: return 1;
    case ONE_PI: return 0;
    case THREE_PI_O2: return -1;
  }
  return 0;
}

int projectColTheta(){
  return projectCol( thetaState );
}

int projectRowTheta(){
  return projectRow( thetaState );
}

int projectColPan(){
  return projectCol( getPanTheta() );
}

int projectRowPan(){
  return projectRow( getPanTheta() );
}
