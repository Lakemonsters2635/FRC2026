package frc.robot;

public class Constants {
    public static final double kPModuleTurningController = 0.5; // 0.5
  public static final double kPModuleDriveController = 0; // added random value for test
    public static final double kDriveEncoderDistancePerPulse = 0.0001 / 0.002706682950506; //TODO: Need to TEST
     public static final double kMaxSpeedMetersPerSecond = 6.0; // TODO: Need to Test

  public static final double kMaxSpeedMetersPerSecond = 6.0; // 4.0;
  // TODO: 'EncoderDistancePerPulse' should be calculated based on the gearing and wheel diameter
  public static final double kWheelDiameterMeters = Units.inchesToMeters(3.5); // 3.5 inch wheels
  public static final double kDriveEncoderDistancePerPulse = 0.0001 / 0.002706682950506;

  public static final double maxModuleLinearSpeed = 1.75; // Irrelevant used for createPath
  public static final double maxModuleLinearAccelaration = 8;
  public static final double kMaxModuleAngularSpeedRadiansPerSecond = 3 * Math.PI;
  public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 12 * Math.PI;

  public static final double DRIVETRAIN_WHEELBASE_WIDTH = Units.inchesToMeters(22); // 26.625
  public static final double DRIVETRAIN_WHEELBASE_LENGTH = Units.inchesToMeters(26); // 19.625

  public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 5;
  public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 6;
  public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER = 2;

  public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 7;
  public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 8;
  public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER = 0;

  public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 3; // TODO: set correct ports 8
  public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 4; // 7
  public static final int DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER = 1;

  public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 1; // TODO: set correct ports 2
  public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 2; // 1
  public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER = 3;

  // SWERVE MODULE STATES
  public static final int FRONT_LEFT_MODULE_STATE_INDEX = 0;
  public static final int FRONT_RIGHT_MODULE_STATE_INDEX = 1;
  public static final int BACK_LEFT_MODULE_STATE_INDEX = 2;
  public static final int BACK_RIGHT_MODULE_STATE_INDEX = 3;

  // ANGLE OFFSETS
  public static final double FRONT_LEFT_ANGLE_OFFSET =
      Math.toRadians(-13 - 90 - 2.5 + 180); // TODO: set correct values
  public static final double FRONT_RIGHT_ANGLE_OFFSET = Math.toRadians(-53 + 90 - 229 + 45);
  public static final double BACK_LEFT_ANGLE_OFFSET = Math.toRadians(-14 + 90 - 40 + 45 + 180);
  public static final double BACK_RIGHT_ANGLE_OFFSET = Math.toRadians(75 - 90 + 3 + 180);

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
  }

