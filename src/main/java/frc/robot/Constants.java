package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
  public static final double kPModuleTurningController = 0.5; // 0.5
  public static final double kPModuleDriveController = 0; // added random value for test
  public static final double kDriveEncoderDistancePerPulse = 0.0001 / 0.002706682950506; //TODO: Need to TEST
  public static final double kMaxSpeedMetersPerSecond = 6.0; // TODO: Need to Test

      // joystick channels
  public static final int RIGHT_JOYSTICK_CHANNEL = 1;
  public static final int LEFT_JOYSTICK_CHANNEL = 0;

  // TODO: 'EncoderDistancePerPulse' should be calculated based on the gearing and wheel diameter
  public static final double kWheelDiameterMeters = Units.inchesToMeters(3.5); // TODO: check actual measurement


  public static final double maxModuleLinearSpeed = 1.75; // Irrelevant used for createPath
  public static final double maxModuleLinearAccelaration = 8;
  public static final double kMaxModuleAngularSpeedRadiansPerSecond = 3 * Math.PI;
  public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 12 * Math.PI;

  public static final double DRIVETRAIN_WHEELBASE_WIDTH = Units.inchesToMeters(22); // TODO； check for correct measurements
  public static final double DRIVETRAIN_WHEELBASE_LENGTH = Units.inchesToMeters(26); // TODO: check for correct measurements

 
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

  // FRONT LEFT
  public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 7; //1
  public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER = 0; //1
  public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 6; //2
  public static final double FRONT_LEFT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-100+180); //3.0346

  // FRONT RIGHT
  public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 1; //7
  public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER = 2; //0
  public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 0; //8
  public static final double FRONT_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(146+180); //2.9835

  // BACK LEFT
  public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 5; //3
  public static final int DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER = 1; //3
  public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 4; //10
  public static final double BACK_LEFT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-13.2); // 3.0775

  // BACK RIGHT
  public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 3; //5
  public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER = 3;//2
  public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 2; //6
  public static final double BACK_RIGHT_ANGLE_OFFSET_COMPETITION = Math.toRadians(-95.3); //3.01

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
  

   //Tipping Constants
  public static final double TIPPING_ANGLE_THRESHOLD = 5;
  public static final int WINDOW_SIZE = 10;
  public static final double SMOOTHING_FACTOR = 0.9;
  public static final double GRAVITY_ACCEL = 1; 
  public static final double GRAVITY_ACCEL_SQUARED = GRAVITY_ACCEL * GRAVITY_ACCEL;
  public static final double PITCH_NOSE_DOWN_PROPORTION_CONSTANT = 0.03; //TODO: tune this value these are percentages
  public static final double PITCH_NOSE_UP_PROPORTION_CONSTANT = 0.03; //TODO: tune this value

  public static final double ROLL_LEFT_PROPORTION_CONSTANT = 0.05; //TODO: tune this value
  public static final double ROLL_RIGHT_PROPORTION_CONSTANT = 0.05; //TODO: tune this value

  public static final int NOSE_DOWN_PITCH = 1;
  public static final int RIGHT_ROLL = 1;
}