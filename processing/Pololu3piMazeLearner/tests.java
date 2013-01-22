// Unit tests

class Tests {

  static final double PI = Math.PI;

  static void check(String msg, boolean test) {
    if (!test) {
      System.out.println("Test \"" + msg + "\" failed!");
      System.exit(0);
    }
  }

  static void DOUBLES_EQUAL(String msg, double expected, double actual, double tol) {
    check(msg,Math.abs(expected-actual)<=tol);
  }

  static void DOUBLES_EQUAL(String msg, double expected, double actual) {
    DOUBLES_EQUAL(msg, expected, actual, 1e-5);
  }

  static void test_twist1() {
    // just move forward with speed 1, for two seconds
    Pose2 expected = new Pose2(2, 0, 0);
    Twist2 twist = new Twist2(1, 0, 0);
    Pose2 actual = twist.expmap(2);
    DOUBLES_EQUAL("test_twist1 x", expected.x(), actual.x());
    DOUBLES_EQUAL("test_twist1 y", expected.y(), actual.y());
    DOUBLES_EQUAL("test_twist1 theta", expected.theta(), actual.theta());
  }

  static void test_twist2() {
    // quarter turn
    Pose2 expected = new Pose2(1, 1, PI/2);
    Twist2 twist = new Twist2(PI/2, 0, PI/2);
    Pose2 actual = twist.expmap(1);
    DOUBLES_EQUAL("test_twist2 x", expected.x(), actual.x());
    DOUBLES_EQUAL("test_twist2 y", expected.y(), actual.y());
    DOUBLES_EQUAL("test_twist2 theta", expected.theta(), actual.theta());
  }

  static void test_twist3() {
    // quarter turn, for two secons = half turn
    Pose2 expected = new Pose2(-8.742278E-8, 2, PI);
    Twist2 twist = new Twist2(PI/2, 0, PI/2);
    Pose2 actual = twist.expmap(2);
    DOUBLES_EQUAL("test_twist3 x", expected.x(), actual.x());
    DOUBLES_EQUAL("test_twist3 y", expected.y(), actual.y());
    DOUBLES_EQUAL("test_twist3 theta", expected.theta(), actual.theta());
  }

  static void test_compose() {
    // two quarter turns = half turn 
    Pose2 expected = new Pose2(0, 2, PI);
    Pose2 pose = new Pose2(1, 1, PI/2);
    Pose2 actual = pose.compose(pose);
    DOUBLES_EQUAL("test_compose x", expected.x(), actual.x());
    DOUBLES_EQUAL("test_compose y", expected.y(), actual.y());
    DOUBLES_EQUAL("test_compose theta", expected.theta(), actual.theta());
  }

  static void run() {
    test_twist1();
    test_twist2();
    test_twist3();
    test_compose();
  }
}

