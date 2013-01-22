/*
 * Pololu3piSimpleFSM
 */

// Initializes the 3pi, displays a welcome message, calibrates, and
// plays the initial music.  This function is automatically called
// by the Arduino framework at the start of program execution.
void setup()
{
  initialize3pi();
  initFSM();
}

// The main function.  This function is repeatedly called by
// the Arduino framework.
void loop()
{
  step();
}


