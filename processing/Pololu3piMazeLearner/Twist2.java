/**
 * A 2D twist (2D linear and 1D angular velocity)
 */
class Twist2 {

  static final double PI = Math.PI;
  double vx_, vy_, omega_;

  // Constructor
  Twist2(double vx, double vy, double omega) {
    vx_=vx;
    vy_=vy;
    omega_=omega;
  }

  /// get x
  double vx() { 
    return vx_;
  }

  /// get y
  double vy() { 
    return vy_;
  }

  /// get omega
  double omega() { 
    return omega_;
  }

  // Exponential map
  Pose2 expmap(double T) {
    double theta = omega_*T;
    while (theta>PI) theta -= 2*PI;
    while (theta<=-PI) theta += 2*PI;
    if (Math.abs(theta)<1e-10) {
      return new Pose2(T*vx_, T*vy_, theta);
    }
    else {
      // in the frame aligned with velocity v, the arc ends at dx,dy after T seconds
      // the radius of the curve is R = v/omega, the angle subtended is theta
      double dx = /* v */ Math.sin(theta)/omega_, dy = /* v */ (1.0-Math.cos(theta))/omega_;
      // now rotate from that frame into robot frame, cos = vx/v, sin = xy/v
      // note that above and below we omit multiplying, resp. dividing by v to save CPU 
      double x = (vx_/* /v */)*dx - (vy_/* /v */)*dy;
      double y = (vy_/* /v */)*dx + (vx_/* /v */)*dy;
      return new Pose2(x, y, theta);
    }
  }

  // print
  void print() {
    System.out.println(String.format("vx:%f m/s, vy:%f m/s, omega:%f rad/sec",vx_, vy_,omega()));
  }

  // prettPrint
  void prettPrint() {
    System.out.println(String.format("vx:%-3.1f cm/s, vy:%-3.1f cm/s, omega:%-4.1f degrees/sec",vx_*100, vy_*100,omega()*180/PI));
  }
}

