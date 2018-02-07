//======================================================================================
// Path Finder!
//======================================================================================
boolean pathFindFromCurrent(int targCol, int targRow) {

  pathTargetCol = targCol;
  pathTargetRow = targRow;

  boxForgive = false;

  //(SectorPath){ nextPoint.col, nextPoint.row, firstPoint.inHeading, firstPoint.outAction };
  subTarget = (SectorPath) {
    valueToSectorMM( x ), valueToSectorMM( y ), thetaState, ACTION_NONE
  };

  boolean success = pathFindRecursive( &subTarget );
  worldPassReset();
  
  if (!success) {
    boxForgive = true;
    subTarget = (SectorPath) {
      valueToSectorMM( x ), valueToSectorMM( y ), thetaState, ACTION_NONE
    };
    success = pathFindRecursive( &subTarget );
    worldPassReset();
  }

  Serial.print("Subtarget: "); Serial.print(subTarget.col); Serial.println( subTarget.row);

  if (subTarget.col > 5)
    subTarget.col = 4;
  else if (subTarget.col < 0)
    subTarget.col = 0;

  if (subTarget.row > 5)
    subTarget.row = 4;
  else if (subTarget.row < 0)
    subTarget.row = 0;

  Serial.print("\t Modified Subtarget: "); Serial.print(subTarget.col); Serial.println(subTarget.row);

  return success;
}

boolean pathFindRecursive(SectorPath *thisSector) {

  //Check if this is even a valid position
  if ( !inSectors(thisSector->col, thisSector->row) || getSectorPass(thisSector->col, thisSector->row) == PASSED_BLOCKED ) {
    resetSectorPassPlan(thisSector->col, thisSector->row);
    return false;
  }

  //Check if we hit the target
  if ( thisSector->col == pathTargetCol && thisSector->row == pathTargetRow) {
    //If we did, set this action as STOP
    thisSector->outAction = ACTION_STOP;
    //Set our subtarget to this
    subTarget.col = thisSector->col;
    subTarget.row = thisSector->row;
    resetSectorPassPlan(thisSector->col, thisSector->row);
    //Spread the word!
    return true;
  }

  //Mark our postion on the pass grid
  setSectorPassPlan( thisSector->col, thisSector->row);

  boolean success = false;

  //These track the direction indicies
  //Since we came from one direction,
  //it naturally follows that there are only three we can take
  //Thus, three variables
  byte headings[3] = {255, 255, 255};

  //And we only need to track the low and mid scores
  //Since we only care about their positions relative to each other;
  byte scores[3] = {255, 255, 255};
  byte genScore = 255;

  //Now, probe our four options:
  for (int probeHead = 0; probeHead < THETA_INCREMENTS; probeHead++) {

    genScore = headingScore(
                 thisSector->col + projectCol(probeHead), thisSector->row + projectRow(probeHead),
                 thisSector->col, thisSector->row, thisSector->inHeading, probeHead
               );

    //if the score is valid
    if ( genScore != 255 ) {
      //if it's lower than the lowest, or equal
      if ( genScore == scores[0] || min(genScore, scores[0]) == genScore ) {
        //boot the middle to the highest
        headings[2] = headings[1]; scores[2] = scores[1];
        //boot the lowest to middle
        headings[1] = headings[0]; scores[1] = scores[0];
        //set the lowest
        headings[0] = probeHead; scores[0] = genScore;
      }
      //else, if it's lower than the middle, or equal
      else if ( genScore == scores[1] || min(genScore, scores[1]) == genScore ) {
        //boot the middle to the highest
        headings[2] = headings[1]; scores[2] = scores[1];
        //set the middle
        headings[1] = probeHead; scores[1] = genScore;
      }
      //else, if it's lower than the highest, or equal
      else if ( genScore == scores[2] || min(genScore, scores[2]) == genScore )
        //set the highest
        headings[2] = probeHead; scores[2] = genScore;
    }
  }

  SectorPath nextSector = (SectorPath) {
    0, 0, 0, ACTION_NONE
  };

  for (int i = 0; i < 3 && !success; i++) {
    if (headings[i] != 255 && scores[i] != 255) {
      thisSector->outAction = headingsToAction( thisSector->inHeading, headings[i] );

      nextSector.col = thisSector->col + projectCol( headings[i] );
      nextSector.row = thisSector->row + projectRow( headings[i] );
      nextSector.inHeading = actionToNewHeading( thisSector->inHeading, thisSector->outAction );
      nextSector.outAction = ACTION_NONE;

      success = pathFindRecursive( &nextSector );
    }
  }

  //If we succeeded
  if (success)
    //And this sector's action is a waypoint action
    if (thisSector->outAction >= ACTION_STOP)
      //And its not the start sector
      if ( thisSector->col != valueToSectorMM( x ) || thisSector->row != valueToSectorMM( y ) ) {
        #ifdef DEBUG
        Serial.println("subtargetReassign"); Serial.print(thisSector->col); Serial.print(thisSector->row);
        #endif
        subTarget.col = thisSector->col; subTarget.row = thisSector->row;
      }

  //Finally, undo the marking so we can use the pass grid later
  resetSectorPassPlan(thisSector->col, thisSector->row);

  return success;
}

byte headingScore(int col, int row,  int currCol, int currRow, int oldHeading, int newHeading) {
  //If a path isn't possible, reject it;
  if ( !isPathPossible( col, row ) )
    return 255;

  //if we are considering high probability squares dangerous, and this is high prob, reject.
  if ( !boxForgive && statGrid[ row ][ col ] >= 0.50)
    return 255;

  //Base score
  byte score = 1;

  //If the parities of the heading don't match, that means we have to turn - that's worth two points.
  if ( (oldHeading % 2) != (newHeading % 2) )
    score += 2;

  //If the sector is an unknown, we multiply the blockage certainty by 10 and add it to the score
  if ( boxForgive && passGrid[ row ][ col ] == PASSED_UNSURE )
    score += byte( 10 * statGrid[ row ][ col ] );

  //If the sector is an unknown, and we are in GoHome or GoIncline, unknown spaces cost more
  if ( machineState == GOHOME && passGrid[ row ][ col ] != PASSED_OPENED )
    score += 3;

  //If, relative to our last position, we're moving away from the goal, we add 4.
  if ( abs(pathTargetCol - currCol) - abs(pathTargetCol - col ) < 0  ||
       abs(pathTargetRow - currRow) - abs(pathTargetRow - row ) < 0 )
    score += 4;

  return score;
}
