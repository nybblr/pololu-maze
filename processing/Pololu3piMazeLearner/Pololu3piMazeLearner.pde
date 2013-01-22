/*
 * Pololu3piMazeRunner
 * More sophisticated finite state machine that runs a maze
 */

// Initializes the 3pi simulation and the finite state machine
// This function is automatically called
// by the Processing framework at the start of program execution.
int speedFactor_; //speeds are divided by this number (>=1) and time intervals are mutiplied by it
void setup()
{
  initialize3piSimulator();
  initFSM();
  speedFactor_ = 2; 
}

// The main function.  This function is repeatedly called by
// the Processing framework.
void draw()
{
  drawWorld(); // draw the world
  step();      // step the finite state machine
  update();    //update the simulated robot pose
}

