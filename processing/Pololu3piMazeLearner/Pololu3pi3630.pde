// Georgia Tech CS 3630
// Frank Dellaert, Jan 2013
// These are all the functions available both in simulator and on 3pi robot

// return the position of the line with respect to the robot:
// 0 = to the left, 2000 = right under robot, 4000 = to the right
// code copied from PololuQTRSensors.cpp
int readLine() {
  int[] sensor_values = readSensors();
  int avg = 0;
  int sum = 0;
  boolean on_line=false;

  for (int i=0;i<5;i++) {
    int value = sensor_values[i];

    // keep track of whether we see the line at all
    if (value > 200) {
      on_line = true;
    }

    avg += (long)(value) * (i * 1000);
    sum += value;
  }

  if (!on_line)
  {
    // If it last read to the left of center, return 0.
    if (lastValue_ < 4*1000/2)
      return 0;

    // If it last read to the right of center, return the max.
    else
      return 4*1000;
  }

  lastValue_ = avg/sum;

  return lastValue_;
}


// Set the wheel speeds, with 255,255 1m/s !
void setSpeeds(int left, int right) {
  robot.setSpeeds(left, right);
}

// lineType, below, returns one of these constants:
final int DEAD_END = 0; // no line visible, not intersection => dead end
final int INTERSECTION = 1; // Found an intersection, can also be used for goal detection by going straight a bit
final int STRAIGHT = 2; // Just a straight line.
final int UNKNOWN = 3; // Just a straight line.

// Helper routine to examine line 
int lineType() {
  int[] sensors = readSensors();
  // We use the inner three sensors (1, 2, and 3) for
  // determining whether there is a line straight ahead, and the
  // sensors 0 and 4 for detecting lines going to the left and right.
  if (sensors[0] > 200 || sensors[4] > 200)
    return INTERSECTION;
  else if (sensors[1] < 100 && sensors[2] < 100 && sensors[3] < 100)
    return DEAD_END;
  else if (sensors[1] > 200 || sensors[2] > 200 || sensors[3] > 200)
    return STRAIGHT;
  else 
    return UNKNOWN;
}

int foundGoal() {
  int[] sensors = readSensors();
  if (sensors[0] > 200 && sensors[1] > 200 && sensors[2] > 200 && sensors[3] > 200 && sensors[4] > 200)
    return 1;
  return 0;
}

final int FOUND_LEFT = -1;
final int FOUND_RIGHT = 1;
final int FOUND_NONE = 0;

// Examine intersection
int intersectionType() {
  int[] sensors = readSensors();
  // Check for left and right exits.
  if (sensors[0] > 100) {
    return FOUND_LEFT;
  }
  else if (sensors[4] > 100){
    return FOUND_RIGHT;
  }
  else {
    return FOUND_NONE;
  }
}

// print a message on console or LCD
void printMessage(String message) {
  println(message);
}

// print a message to console in Java or to LCD on Pololu with state index
void printState(char state) {
  println("State "+state);
}

// Turn off motors and wait for Button press
void waitForButtonPress() {
  while (!mousePressed) {
  }
  while (mousePressed) {
  }
}

// PID variables
int lastError_ = 0;
int integral_ = 0;

// PID controller for line following
void followPID(int switched)
{
  // If we just entered this state, reset PID controller
  if (switched==1) {
    int lastError_ = 0;
    int integral_=0;
  }

  // The "error" term should be 0 when we are on the line.
  int position = readLine();
  int error = (int)position - 2000;

  // Compute the derivative (change) and integral_ (sum) of the
  // position.
  int derivative = error - lastError_;
  integral_ += error;

  // Remember the last position.
  lastError_ = error;

  // Compute the difference between the two motor power settings,
  // m1 - m2.  If this is a positive number the robot will turn
  // to the right.  If it is a negative number, the robot will
  // turn to the left, and the magnitude of the number determines
  // the sharpness of the turn.  You can adjust the constants by which
  // the error, integral_, and derivative terms are multiplied to
  // improve performance.
  //  int power_difference = error/20 + integral_/10000 + derivative*3/2; // Pololu originals
  int power_difference = error/100 + integral_/50000 + derivative*3/10;

  // Compute the actual motor settings.  We never set either motor
  // to a negative value.
  int maximum = 80;
  if (power_difference > maximum)
    power_difference = maximum;
  if (power_difference < -maximum)
    power_difference = -maximum;

  // original Pololu code:
  setSpeeds((maximum + power_difference/2)/speedFactor_, (maximum - power_difference/2)/speedFactor_);
}

//================================================================================================
// The path_ variable will store the path_ that the robot has taken.  It
// is stored as an array of characters, each of which represents the
// turn that should be made at one intersection in the sequence:
//  'L' for left
//  'R' for right
//  'S' for straight (going straight through an intersection)
//  'B' for back (U-turn)
//================================================================================================
char[] path_ = new char[1];
int pathLength_ = 0; // the length of the path_
int maxPathLength_ = 1;

void addTurn(char turn) {
  if (pathLength_ == maxPathLength_) {
    maxPathLength_ *= 2;
    char[] temp = new char[maxPathLength_];
    for (int i = 0; i < pathLength_; i++) {
      temp[i] = path_[i];
    }
    path_ = temp;
  }
  path_[pathLength_] = turn;
  pathLength_ ++;
  simplifyPath();
  displayPath();
}

  // Displays the current path_ on the LCD, using two rows if necessary.
  void displayPath()
  {
    for (int i=0;i<pathLength_;i++) print(path_[i]);
    println("");
  }

  // elmininates unnecessary paths by replacing any sequence *B* with the net turn
  void simplifyPath() {
      // only simplify the path if the second-to-last turn was a 'B'
      if(pathLength_ < 3 || path_[pathLength_-2] != 'B')
          return;
      int total_angle = 0;
      int i;
      for(i=1;i<=3;i++)
      {
          switch(path_[pathLength_-i])
          {
          case 'R':
              total_angle += 90;
              break;
          case 'L':
              total_angle += 270;
              break;
          case 'B':
              total_angle += 180;
              break;
          }
      }
   
      // Get the angle as a number between 0 and 360 degrees.
      total_angle = total_angle % 360;
   
      // Replace all of those turns with a single one.
      switch(total_angle)
      {
      case 0:
          path_[pathLength_ - 3] = 'S';
          break;
      case 90:
          path_[pathLength_ - 3] = 'R';
          break;
      case 180:
          path_[pathLength_ - 3] = 'B';
          break;
      case 270:
          path_[pathLength_ - 3] = 'L';
          break;
      }
  
      // The path is now two steps shorter.
      pathLength_ -= 2;
  }

