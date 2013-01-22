// Variables initialized here
static final double INCH = 0.0254;
Robot robot = new Robot(0, 0, 0);
int fps = 100;
double dt = 1.0/(double)fps;
double  s_ = 6/INCH; // scale, in pixels/meter
int u0_ = 100, v0_ = 100; // image coordinate of 0,0
int lastValue_ = 2000; // temporary in readline
Tests tests; // static class

// Variables initialized in initialize
PImage maze_;

// One time setup
void initialize3piSimulator () {
  size(600, 600);
  frameRate(fps);

  // run unit tests first
  tests.run();

  maze_ = loadImage("maze2.png");
  maze_.filter(BLUR, 1);
}

// Small inner class
class Point {
  int u_, v_;
  Point(int u, int v) {
    u_=u;
    v_=v;
  }
}

/// Convert to pixel coordinate
Point transform_to(Point2 p) {
  return new Point(u0_+(int)(0.5+s_*p.x()), v0_-(int)(0.5+s_*p.y()));
}


/// Draw
void drawRobot() {
  Point p = transform_to(robot.pose().translation());
  int d = (int)(s_*0.09398);
  fill(0, 150, 0);
  ellipse(p.u_, p.v_, d, d);
}

/// Draw robot sensors
void drawSensors() {
  int d = (int)(s_*0.01);
  for (int i=0;i<5;i++) {
    Point p = transform_to(robot.sensorPosition(i));
    fill((4-i)*63, i*63, 0);
    ellipse(p.u_, p.v_, d, d);
  }
}

/// Return sensor measurements
int[] readSensors() {
  int [] sensor_values = new int[5]; 
  for (int i=0;i<5;i++) {
    Point p = transform_to(robot.sensorPosition(i));
    sensor_values[i] = (int)(255-brightness(get(p.u_, p.v_)))*1000/255;
  }
  return sensor_values;
}

/// Draw Sensor Bargraph like on Pololu LCD
void drawSensorBars() {
  int[] sensor_values = readSensors();
  // interprets the first two parameters of rect() as the shape's center point, 
  // while the third and fourth parameters are its width and height.
  rectMode(CENTER);
  // draw bars
  fill(255, 0, 0);
  for (int i=0;i<5;i++)
    rect(i*20, 30, 15, sensor_values[i]/20);
  // draw position as computed by readLine
  double position = (double)(readLine())/1000;
  fill(0, 255, 0);
  rect((int)(position*20), 25, 2, 50);
}

/// Draw motor speeds
void drawMotorSpeeds() {
  rectMode(CENTER);
  // draw bars
  fill(0, 0, 255);
  rect(190, 25, 15, (int)robot.phiL());
  rect(210, 25, 15, (int)robot.phiR());
}

// draw world
// needs to be done before update or sensors will not work properly
void drawWorld() {
  // draw the world and the sensor readings
  background(255);
  image(maze_, u0_-27, v0_-27);
  drawSensorBars();
}

// draw then update robot
void update() {
  drawRobot();
  drawSensors();
  drawMotorSpeeds();

  // Update the robot for the next time step
  robot.update(dt);
}

