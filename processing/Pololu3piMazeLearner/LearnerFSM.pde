//================================================================================================
// define states here, do not repeat characters 
//================================================================================================
final char START = 'S'; // always needs to be defined !
final char FOLLOW = 'F';
final char U_TURN = 'U';
final char ENTER = 'E';
final char TURN_LEFT = 'L';
final char TURN_RIGHT = 'R';
final char GO_STRAIGHT = 'T';
final char GOAL = 'G';

int intersection_type = 0; //left, right, straight

int current = 0; // current position in path

//================================================================================================
// Check for triggers that change state, no motor commands here
//================================================================================================
void checkTriggers(int elapsed) {

  switch(state_) {

  case START:  
    // get out of start circle
    if (elapsed>400) switchToState(FOLLOW);
    break;

  case FOLLOW:
    // allo some time to get back on the line
    if (elapsed>50) {    
      if (lineType()==INTERSECTION) switchToState(ENTER);
      if (lineType()==DEAD_END) switchToState(U_TURN);
    }
    break;

  case ENTER:  
    // Drive straight a bit.
    // after we're GOAL, decide which way to turn
    if (elapsed>200) {
      if (foundGoal() == 1) {
        switchToState(GOAL);
        break;
      }
      if (intersection_type != FOUND_LEFT)
        if (lineType() == STRAIGHT)
          intersection_type = STRAIGHT;
      switch(intersection_type) {
        case FOUND_LEFT:
          switchToState(TURN_LEFT);
          break;
        case STRAIGHT:
          switchToState(GO_STRAIGHT);
          break;
        case FOUND_RIGHT:
          switchToState(TURN_RIGHT);
          break;
        case FOUND_NONE:
          switchToState(U_TURN);
          break;
      }
    }
    break;

  case TURN_LEFT:  
    if (elapsed>180) switchToState(FOLLOW);
    break;

  case TURN_RIGHT:  
    if (elapsed>180) switchToState(FOLLOW);
    break;
    
  case GO_STRAIGHT:
    if (elapsed > 50) switchToState(FOLLOW);
    break;
    
  case U_TURN:
    if (elapsed > 360) switchToState(FOLLOW);
    break; 

  case GOAL:  
    // Wait for button-press, then restart everything
    waitForButtonPress();
    current=0;
    switchToState(START);
    break;
  } // switch
}

//================================================================================================
// now execute current behavior
//================================================================================================
void executeBehavior(int elapsed) {
  switch(state_) {
  case START:  
    setSpeeds(60, 60);
    break;

  case FOLLOW:  
    followPID(switched_);
    break;

  case ENTER:  
    // Note that we are slowing down - this prevents the robot
    // from tipping forward too much.
    if (switched_==1) {
      setSpeeds(50, 50);
      intersection_type = intersectionType();
    }
    break;

  case TURN_LEFT:  
    if (switched_==1) {
      setSpeeds(-80, 80);
      addTurn('L');
    }
    break;

  case TURN_RIGHT:  
    if (switched_==1){
      setSpeeds(80, -80);
      addTurn('R');
    }
    break;
    
  case GO_STRAIGHT:  
    if (switched_==1){
      setSpeeds(60, 60);
      addTurn('S');
    }
    break;
  
  case U_TURN:  
    if (switched_==1){
      setSpeeds(80, -80);
      addTurn('B');
    }
    break;
    
  case GOAL:  
    if (switched_==1){ 
      setSpeeds(0, 0);
      addTurn('G');
    }

  } // switch
}


//================================================================================================



