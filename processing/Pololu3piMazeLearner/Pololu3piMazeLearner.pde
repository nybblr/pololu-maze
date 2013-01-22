/*
 * Pololu3piMazeRunner
 * More sophisticated finite state machine that runs a maze
 */

// Initializes the 3pi simulation and the finite state machine
// This function is automatically called
// by the Processing framework at the start of program execution.
void setup()
{
  initialize3piSimulator();
  initFSM();
}

// The main function.  This function is repeatedly called by
// the Processing framework.
void draw()
{
  drawWorld(); // draw the world
  step();      // step the finite state machine
  update();    //update the simulated robot pose
}

