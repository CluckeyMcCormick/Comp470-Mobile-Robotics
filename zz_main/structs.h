
//======================================================================================
// Theta Management
//======================================================================================

#define THETA_INCREMENTS 4
enum IncrementalTheta {ZERO_PI, PI_O2, ONE_PI, THREE_PI_O2};

IncrementalTheta radianLeft(IncrementalTheta rad){
  switch(rad){
    default:
    case ZERO_PI: return PI_O2;
      break;
    case PI_O2: return ONE_PI;
      break;
    case ONE_PI: return THREE_PI_O2; 
      break;
    case THREE_PI_O2: return ZERO_PI;
      break;
  }
}

IncrementalTheta radianRight(IncrementalTheta rad){
  switch(rad){
    default:
    case ZERO_PI: return THREE_PI_O2;
      break;
    case PI_O2: return ZERO_PI;
      break;
    case ONE_PI: return PI_O2;
      break;
    case THREE_PI_O2: return ONE_PI;
      break;
  }
}

double radianToValue( IncrementalTheta rad ){
  switch(rad){
    default:
    case ZERO_PI: return 0;
      break;
    case PI_O2: return PI / 2;
      break;
    case ONE_PI: return PI;
      break;
    case THREE_PI_O2: return (3 * PI) / 2;
      break;
  }
}

typedef struct {
  int col; //The column of this sector
  int row; //The row of this sector
  int inHeading; //The heading when we entered this sector
  int outAction; //The action we took to leave this sector
} SectorPath;
