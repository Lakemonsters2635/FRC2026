// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

@Logged
public class SwerveModule {
  private final TalonFX m_driveMotor;
  public final TalonFX m_turningMotor;

  // private final SparkMaxConfig m_driveMotorConfig;
  // private final SparkMaxConfig m_turningMotorConfig;

  // public final StatusSignal<Angle> m_driveEncoder;
  public final AnalogInput m_turningEncoderInput;
  public final AnalogEncoder m_turningEncoder;

  private double turningMotorOffset;

  private final PIDController m_drivePIDController = new PIDController(0.002, 0, 0.00);
  private final PIDController m_turningPIDController = new PIDController(Constants.kPModuleTurningController, 0, 0.0001);

  public NetworkTableEntry t_turningEncoder;

  private double m_driveMotorGain;
  /**
   * Constructs a SwerveModule.
   * 
   * TODO: drive PID not used.  the PID constants are all zero.  should document why.
   * TODO: update note in setDesiredState() for drive Output regarding PID constants for drive if they are updated.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int analogEncoderPort,
      double turningMotorOffset,
      double driveMotorGain // tuning motor module
      ) {

    m_driveMotor = new TalonFX(driveMotorChannel, new CANBus("CANivore"));
    m_turningMotor = new TalonFX(turningMotorChannel,  new CANBus("CANivore"));

    m_driveMotor.setNeutralMode(NeutralModeValue.Brake);
    m_turningMotor.setNeutralMode(NeutralModeValue.Brake);

    this.turningMotorOffset = turningMotorOffset;

    m_driveMotorGain = driveMotorGain;

    /**
     * Parameters can be set by calling the appropriate Set method on the CANSparkMax object
     * whose properties you want to change
     * 
     * Set methods will return one of three REVLibError values which will let you know if the 
     * parameter was successfully set:
     *  REVLibError.kOk
     *  REVLibError.kError
     *  REVLibError.kTimeout
     * https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Get%20and%20Set%20Parameters/src/main/java/frc/robot/Robot.java
     */
    // if(m_driveMotorConfig.setIdleMode(IdleMode.kBrake) != REVLibError.kOk){
    //   SmartDashboard.putString("Idle Mode", "Error");
    // }

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_turningEncoderInput = new AnalogInput(analogEncoderPort);
    m_turningEncoder = new AnalogEncoder(m_turningEncoderInput);
    // m_driveEncoder = m_driveMotor.getPosition();

    // System.out.println(m_driveMotor.getPosition().getValueAsDouble());
    // System.out.println(m_turningEncoder.get());
    
    // SmartDashboard.putNumber("m_driveEncoder.get", m_driveMotor.getPosition().getValueAsDouble());
    // SmartDashboard.putNumber("m_turningEncoder.get", m_turningEncoder.get());
    
    // Deprecated
    // m_driveEncoder.setPositionConversionFactor(Constants.kDriveEncoderDistancePerPulse);
    // m_driveEncoder.setVelocityConversionFactor(Constants.kDriveEncoderDistancePerPulse/60.0);


    // Set whether drive encoder should be reversed or not
    // m_driveEncoder.setReverseDirection(driveEncoderReversed);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.

    // Set whether turning encoder should be reversed or not
    // m_turningEncoder.setReverseDirection(turningEncoderReversed);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void updateSwerveTable() {
    t_turningEncoder.setDouble(Math.toRadians(m_turningEncoder.get()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_driveMotor.getPosition().getValueAsDouble() * Constants.kDriveEncoderDistancePerPulse, new Rotation2d(getTurningEncoderRadians()));
  }

  public double getTurningEncoderRadians(){
    double angle = (1.0 - (m_turningEncoderInput.getVoltage()/RobotController.getVoltage5V())) * 2.0 * Math.PI + turningMotorOffset;
    angle %= 2.0 * Math.PI;
    if (angle < 0.0) {
        angle += 2.0 * Math.PI;
    }

    return angle;
  }

  public double printVoltage() {
    return m_turningEncoderInput.getVoltage();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getVelocity().getValueAsDouble(), new Rotation2d(getTurningEncoderRadians()));
  }

  public double getVelocity() {
    return m_driveMotor.getVelocity().getValueAsDouble();
  }

  public void stop(){
    m_driveMotor.set(0);
    m_turningMotor.set(0);
  }
  
  private static int loopCtr = 0;
  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    // TODO: SwerveModuleState.optimize() is deprecated. Need to implement our own logic.

    // SwerveModuleState state =
    //     SwerveModuleState.optimize(desiredState, new Rotation2d(getTurningEncoderRadians()));

    SwerveModuleState state = desiredState;
    
    state.optimize(new Rotation2d(getTurningEncoderRadians()));

    // Calculate the drive output from the drive PID controller.
    // Note: due to the drive PID constants being zero currently, this driveOutput will
    //       always be zero.
    final double driveOutput = //state.speedMetersPerSecond;
      m_drivePIDController.calculate(m_driveMotor.getVelocity().getValueAsDouble(), state.speedMetersPerSecond);

    // This computes the velocity error regardless of direction of travel
    // such that >0 means too fast and <0 means too slow
   // double velocityError = Math.copySign(state.speedMetersPerSecond - m_driveEncoder.getVelocity(), state.speedMetersPerSecond);


    // String str = String.format("setDesiredState/Verror%d", m_driveMotor.getDeviceId());
    // SmartDashboard.putNumber(str, velocityError);

    final double driveFeedForward = state.speedMetersPerSecond / DrivetrainSubsystem.kMaxSpeed;

    // Calculate the turning motor output from the turning PID controller.
    final var turnOutput =
        m_turningPIDController.calculate(getTurningEncoderRadians(), state.angle.getRadians());

    loopCtr++;
    // if ((loopCtr % 50 == 0) && (m_driveMotor.getDeviceId() == 8))
    // {
      
      // System.out.println(
      //   "Spd: " + Math.round(state.speedMetersPerSecond * 100.) / 100. + 
      //   "  getV(): " + Math.round(m_driveEncoder.getVelocity() * 100.) / 100. + 
      //   "  DO: "+ Math.round(driveOutput * 100.) / 100.
      // );
    // }

    //String str1 = String.format("setDesiredState/Drive%d", m_driveMotor.getDeviceId());
    // SmartDashboard.putNumber(str1, driveOutput);
    //String str2 = String.format("setDesiredState/FF%d", m_driveMotor.getDeviceId());
    // SmartDashboard.putNumber(str2, driveFeedForward);

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(
      Math.max(
        -1.0, 
        Math.min(
          (driveOutput + driveFeedForward) * m_driveMotorGain, 
          1.0
        )
      )
    );
    m_turningMotor.set(turnOutput);
  }
}