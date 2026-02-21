package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class Constants {
    //Shooter Constants
    public static final int SHOOTER_MOTOR_ID_LEFT = 5; 
    public static final int SHOOTER_MOTOR_ID_RIGHT = 6; 
    public static final double SHOOT = 5; //Can go up to 10 volts
    public static final double MOTOR_STOP = 0.0;

    // Transport Constants
    public static final int AGITATOR_MOTOR_TOP_ID = 21; //TODO: change to correct id
    public static final int AGITATOR_MOTOR_BOTTOM_ID = 22; //TODO: change to correct id
    public static final int UPTAKE_MOTOR_ID = 0; //TODO: change to correct id
    //Turret Constants
    public static final int TURRET_MOTOR_ID = 0; //TODO: find id
    public static final int RATIO_SPARKMAX_ROTATION_TO_TURRET = 30; //3 x 10 gear ratios
    public static final int ENCODER_TICS_PER_SPARKMAX_REVOLUTION = 42;

    //Actuator Constnats
    public static final int LEFT_ACTUATOR_ID = 1;
    public static final int RIGHT_ACTUATOR_ID = 0;
  
     // JOYSTICKS
    public static final int LEFT_JOYSTICK_PORT = 0;
    public static final int RIGHT_JOYSTICK_PORT = 1;
  };

