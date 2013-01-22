// Simulated robot
class Robot {

  static final double INCH = 0.0254, MM=0.001; // units
  static final double D = 3.7*INCH; // robot diameter, http://www.pololu.com/catalog/product/975/specs
  static final double R = 16*MM; // wheel radius, http://www.pololu.com/catalog/product/1088 

  Pose2 pose_;
  double phiL_, phiR_; // rotational wheel speeds in rad/sec

  // Constructor
  Robot(double x, double y, double theta) {
    pose_ = new Pose2(x, y, theta);
    phiL_=0;
    phiR_=0;
  }

  // Reset to pose
  void reset(double x, double y, double theta) {
    pose_ = new Pose2(x, y, theta);
    phiL_=0;
    phiR_=0;
  }

  /// get pose
  Pose2 pose() { 
    return pose_;
  }

  /// get x
  double x() { 
    return pose_.x();
  }

  /// get y
  double y() { 
    return pose_.y();
  }

  /// get theta
  double theta() { 
    return pose_.theta();
  }

  /// get phiL
  double phiL() { 
    return phiL_;
  }

  /// get phiR
  double phiR() { 
    return phiR_;
  }

  // set motor speeds, ignoring dynamics/acceleration phase
  final int MAX_RPM=800; // tuned to be similar to 3pi
  void setSpeeds(int left, int right) {
    double S = (MAX_RPM/60)*2*Math.PI/255;
    phiL_ = S*(double)left;
    phiR_ = S*(double)right;
  }

  // Forward kinematics: from wheel speeds to 2D twist, in robot frame
  Twist2 forwardKinematics() {
    double vL = R*phiL_, vR = R*phiR_; // tangential wheel speeds 
    double v = (vR+vL)/2; // forward velocity
    double omega = (vR-vL)/D; // angular velocity
    return new Twist2(v, 0, omega);
  }

  // Update, using Euler integration
  void update(double dt) {
    Twist2 twist = forwardKinematics();
    Pose2 deltaPose = twist.expmap(dt);
    pose_ = pose_.compose(deltaPose);
  }

  /// Return sensor position in world frame
  Point2 sensorPosition(int i) {
    // in 3pi-bottom.jpg, diameter = 538 pixels, = 0.175mm/pixel, middle = 294
    // sensors are at (horizontal,vertical)
    // (300,43) = (0,294-43)*0.175 = (0,44mm)
    // (359,49) = (59,294-49)*0.175 = (10mm,43mm) 
    // (445,87) = (145,294-87)*0.175 = (25mm,36mm)
    // In the robot coordinate frame, though, the X-axis points forward,
    // sopositive x is towards front, positive y towards left 
    switch (i) {
    case 0: // leftmost
      return pose_.transform_from(new Point2(0.036, 0.025));
    case 1: 
      return pose_.transform_from(new Point2(0.043, 0.01));
    case 2: 
      return pose_.transform_from(new Point2(0.044, 0));
    case 3: 
      return pose_.transform_from(new Point2(0.043, -0.01));
    case 4: // rightmost
      return pose_.transform_from(new Point2(0.036, -0.025));
    }
    return new Point2(0,0);
  }
}

