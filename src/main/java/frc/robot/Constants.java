package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
  public static final double kPModuleTurningController = 0.1; // 0.5
  public static final double kPModuleDriveController = 0; // added random value for test
  public static final double kDriveEncoderDistancePerPulse =
      0.0001 / 0.002706682950506; // TODO: Need to TEST
  public static final double kMaxSpeedMetersPerSecond = 6.0; // TODO: Need to Test

  // joystick channels
  public static final int RIGHT_JOYSTICK_CHANNEL = 1;
  public static final int LEFT_JOYSTICK_CHANNEL = 0;

  // TODO: 'EncoderDistancePerPulse' should be calculated based on the gearing and wheel diameter
  public static final double kWheelDiameterMeters =
      Units.inchesToMeters(3.5); // TODO: check actual measurement

  public static final double maxModuleLinearSpeed = 1.75; // Irrelevant used for createPath
  public static final double maxModuleLinearAccelaration = 8;
  public static final double kMaxModuleAngularSpeedRadiansPerSecond = 3 * Math.PI;
  public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 12 * Math.PI;

  public static final double DRIVETRAIN_WHEELBASE_WIDTH =
      Units.inchesToMeters(19.5); // TODO； check for correct measurements
  public static final double DRIVETRAIN_WHEELBASE_LENGTH =
      Units.inchesToMeters(24.75); // TODO: check for correct measurements

  // SWERVE MODULE STATES
  public static final int FRONT_LEFT_MODULE_STATE_INDEX = 1;
  public static final int FRONT_RIGHT_MODULE_STATE_INDEX = 0;
  public static final int BACK_LEFT_MODULE_STATE_INDEX = 3;
  public static final int BACK_RIGHT_MODULE_STATE_INDEX = 2;

  // ANGLE OFFSETS
  public static final double FRONT_LEFT_ANGLE_OFFSET =
      Math.toRadians(-13 - 90 - 2.5 + 180 - 15 - 90 + 90 - 90);
  public static final double FRONT_RIGHT_ANGLE_OFFSET =
      Math.toRadians(-53 + 90 - 229 + 45 + 99 - 90 + 90 - 90+88);
  public static final double BACK_LEFT_ANGLE_OFFSET =
      Math.toRadians(-14 + 90 - 40 + 45 + 180 - 120 - 4 + 90 + 90 - 90);
  public static final double BACK_RIGHT_ANGLE_OFFSET =
      Math.toRadians(75 - 90 + 3 + 180 - 54 + 7 + 90 - 90+82);

  // FRONT LEFT
  public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 1; // 1
  public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER = 1; // 1
  public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 2; // 2

  // FRONT RIGHT
  public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 9; // 7
  public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER = 3; // 0
  public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 10; // 8

  // BACK LEFT
  public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 3; // 3
  public static final int DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER = 0; // 3
  public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 4; // 10

  // BACK RIGHT
  public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 8; // 5
  public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER = 2; // 2
  public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 7; // 6

  // HAT CONSTANTS
  public static final double HAT_POWER_MOVE = 0.1;
  public static final double HAT_POWER_ROTATE = 0.3;

  public static final int HAT_POV_MOVE_LEFT = 270;
  public static final int HAT_POV_MOVE_RIGHT = 90;
  public static final int HAT_POV_MOVE_FORWARD = 0;
  public static final int HAT_POV_MOVE_BACK = 180;
  public static final int HAT_POV_0 = 0; // Left hat up
  public static final int HAT_POV_180 = 180; // Left hat down
  public static final int HAT_POV_ROTATE_LEFT = 270;
  public static final int HAT_POV_ROTATE_RIGHT = 90;

  // Tipping Constants
  public static final double TIPPING_ANGLE_THRESHOLD = 5;
  public static final int WINDOW_SIZE = 10;
  public static final double SMOOTHING_FACTOR = 0.9;
  public static final double GRAVITY_ACCEL = 1;
  public static final double GRAVITY_ACCEL_SQUARED = GRAVITY_ACCEL * GRAVITY_ACCEL;
  public static final double PITCH_NOSE_DOWN_PROPORTION_CONSTANT =
      0.03; // TODO: tune this value these are percentages
  public static final double PITCH_NOSE_UP_PROPORTION_CONSTANT = 0.03; // TODO: tune this value

  public static final double ROLL_LEFT_PROPORTION_CONSTANT = 0.05; // TODO: tune this value
  public static final double ROLL_RIGHT_PROPORTION_CONSTANT = 0.05; // TODO: tune this value

  public static final int NOSE_DOWN_PITCH = 1;
  public static final int RIGHT_ROLL = 1;

  // OBJECT TRACKER SUBSYSTEM
  public static final double CAMERA_TILT =
      26.5; // previous was 37, 30 is for testing post bunnybots
  public static final double[] CAMERA_OFFSET = {3, 13}; // offset = [x, y], In inches
  // VISION AUTO COMMAND
  // TODO: VisionAutoData assumes only a single camera
  public static final int CAM_X_OFFSET = 0;
  public static final int CAM_Y_OFFSET = 0;
  public static final int CAM_ANGLE_OFFSET = 0;
  public static final int SHOOTER_MOTOR_ID_LEFT = 5;
  public static final int SHOOTER_MOTOR_ID_RIGHT = 6;
  public static final double SHOOT = 5; // Can go up to 10 volts
  public static final double MOTOR_STOP = 0.0;
  // Intake Constants
  public static final int INTAKE_MOTOR_ID = 22;
  public static final int IN_VOLTAGE = 12;
  public static final int OUT_VOLTAGE = -12;
  public static final int INTAKE_IN_BUTTON = 8;
  public static final int INTAKE_OUT_BUTTON = 9;
  // Transport Constants
  public static final int AGITATOR_MOTOR_TOP_ID = 13; // TODO: change to correct id
  public static final int AGITATOR_MOTOR_BOTTOM_ID = 12; // TODO: change to correct id
  public static final int UPTAKE_MOTOR_ID = 0; // TODO: change to correct id

  // Turret Constants
  public static final int TURRET_MOTOR_ID = 21; // TODO: find id
  public static final int RATIO_SPARKMAX_ROTATION_TO_TURRET = 30; // 3 x 10 gear ratios
  public static final int ENCODER_TICS_PER_SPARKMAX_REVOLUTION = 42;
  public static final double TURRET_POWER = 2; // TODO: Chang
  public static final double TURRET_ANGLE_RANGE =
      2; // This is the amount of error that the go to pose command has in degrees TODO: Change
  public static final double TURRET_PID_CONSTANT = 0.6;
  public static final double TURRET_ANGLE_MOVE = 4.0;
  public static final double MIN_LIMIT_ROTATION = -70;
  public static final double MAX_LIMIT_ROTATION = 80;
  public static final double APRIL_TAG_AIM_OFFSET = 5;
  // Actuator Constnats
  public static final int LEFT_ACTUATOR_ID = 8;
  public static final int RIGHT_ACTUATOR_ID = 9;

  // JOYSTICKS
  public static final int LEFT_JOYSTICK_PORT = 0;
  public static final int RIGHT_JOYSTICK_PORT = 1;
}
