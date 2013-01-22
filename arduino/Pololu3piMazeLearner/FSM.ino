// Finite State Machine
char state_; // current state
int timer_; // when we last switched
int switched_; // did we just switch?

// initialize finite state machine
void initFSM() {
  state_ = 'S'; // needs to be defined !
  int switched_ = 1;
  int timer_ = millis();
  printState(state_);
}

// Switch state and reset timer
void switchToState(char state) {
  state_ = state;
  switched_ = 1;
  printState(state_);
  // uncomment this to debug on Pololu
  // waitForButtonPress();
  timer_ = millis();
}

// do one time-step
void step() {
  int elapsed = millis()-timer_; // always include this

  // state machine needs to define the two functions below !

  // Check for triggers that change state
  checkTriggers(elapsed);

  if (switched_==1) {
    elapsed = millis()-timer_; // if we did change state, reset elapsed
  }

  // now execute current behavior
  executeBehavior(elapsed);

  switched_ = 0; // always include this to reset this flag
}





