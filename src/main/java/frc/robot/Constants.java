package frc.robot;

public class Constants {
  // Transport
  public static final int AGITATOR_MOTOR_TOP_ID = 21; //TODO: change to correct id
  public static final int AGITATOR_MOTOR_BOTTOM_ID = 22; //TODO: change to correct id
  public static final int UPTAKE_MOTOR_ID = 23; //TODO: change to correct id


  public static final double kPModuleTurningController = 0.5; // 0.5
  public static final double kPModuleDriveController = 0; // added random value for test
  public static final double kDriveEncoderDistancePerPulse = 0.0001 / 0.002706682950506; //TODO: Need to TEST
  public static final double kMaxSpeedMetersPerSecond = 6.0; // TODO: Need to Test
  public static final int INTAKE_MOTOR_ID = 1; // TODO: Fix when we know ID
  public static final double IN_VOLTAGE_ID = 2; // TODO: Fix later when we find voltage
  public static final double OUT_VOLTAGE_ID = -2; //TODO: Fix later when we find voltage
  public static final int MOTOR_STOP_ID = 0; // to stop the motors
  public static final int LEFT_JOYSTICK_ID = 0; // TODO: Change if wrong index
  public static final int RIGHT_JOYSTICK_ID = 1; // TODO: ^^^
  public static final int INTAKE_IN_BUTTON_NUMBER = 4; // TODO: Adjust later
  public static final int INTAKE_OUT_BUTTON_NUMBER = -4; // TODO: Adjust later
  public static final int SHOOTER_MOTOR_ID = 5; //TODO: Change later when we find ID
  public static final int SHOOT_OUT_ID = 2; //TODO: Adjust when we test
     
}
