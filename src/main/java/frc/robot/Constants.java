package frc.robot;

public class Constants {
    public static final double kPModuleTurningController = 0.5; // 0.5
  public static final double kPModuleDriveController = 0; // added random value for test
    public static final double kDriveEncoderDistancePerPulse = 0.0001 / 0.002706682950506; //TODO: Need to TEST
     public static final double kMaxSpeedMetersPerSecond = 6.0; // TODO: Need to Test
     public static final int INTAKE_MOTOR_ID = 1; // TODO: Fix when we know ID
     public static final double IN_VOLTAGE = 2; // TODO: Fix later when we find voltage
     public static final double OUT_VOLTAGE = -2; //TODO: Fix later when we find voltage
     public static final int MOTOR_STOP = 0; // to stop the motors
     public static final int LEFT_JOYSTICK_ID = 0; // TODO: Change if wrong index
     public static final int RIGHT_JOYSTICK_ID = 1; // TODO: ^^^
     public static final int INTAKE_IN_BUTTON = 4; // TODO: Adjust later
     public static final int INTAKE_OUT_BUTTON = 8; // TODO: Adjust later
}
