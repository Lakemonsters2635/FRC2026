// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class SwerveModule {
  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;

  private final AnalogInput m_turningEncoderInput;

  private double turningMotorOffsetRadians;

  private final PIDController m_drivePIDController =
      new PIDController(Constants.kPModuleDriveController, 0, 0);
  ;
  private final PIDController m_turningPIDController =
      new PIDController(Constants.kPModuleTurningController, 0, 0.0001);

  private double m_driveMotorGain;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   * @param analogEncoderPort Analog input port for the turning encoder.
   * @param turningMotorOffsetRadians Offset to add to the turning encoder reading to align with the
   *     module's zero position.
   * @param driveMotorGain Gain to apply to the drive motor output for tuning.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int analogEncoderPort,
      double turningMotorOffsetRadians,
      double driveMotorGain) {

    // VoltageConfigs m_voltageConfigDrive = new VoltageConfigs();
    // m_voltageConfigDrive.SupplyVoltageTimeConstant = 1;
    // VoltageConfigs m_voltageConfigTurning = new VoltageConfigs();
    // m_voltageConfigTurning.SupplyVoltageTimeConstant = 10;

    m_driveMotor = new TalonFX(driveMotorChannel);
    m_turningMotor = new TalonFX(turningMotorChannel);

    // m_driveMotor.getConfigurator().apply(m_voltageConfigDrive);
    // m_turningMotor.getConfigurator().apply(m_voltageConfigTurning);

    m_driveMotor.setNeutralMode(NeutralModeValue.Brake);
    m_turningMotor.setNeutralMode(NeutralModeValue.Brake);

    this.turningMotorOffsetRadians = turningMotorOffsetRadians;

    m_driveMotorGain = driveMotorGain;

    m_turningEncoderInput = new AnalogInput(analogEncoderPort);

    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveMotor.getPosition().getValueAsDouble() * Constants.kDriveEncoderDistancePerPulse,
        new Rotation2d(getTurningEncoderRadians()));
  }

  public double getTurningEncoderRadians() {
    double angle =
        (1.0 - (m_turningEncoderInput.getVoltage() / RobotController.getVoltage5V()))
                * 2.0
                * Math.PI
            + turningMotorOffsetRadians;
    angle %= 2.0 * Math.PI;
    if (angle < 0.0) {
      angle += 2.0 * Math.PI;
    }

    return angle;
  }

  public double getTurningEncoderVoltage() {
    return m_turningEncoderInput.getVoltage();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveMotor.getVelocity().getValueAsDouble(), new Rotation2d(getTurningEncoderRadians()));
  }

  public double getVelocity() {
    return m_driveMotor.getVelocity().getValueAsDouble();
  }

  public void stop() {
    m_driveMotor.set(0);
    m_turningMotor.set(0);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    SwerveModuleState state = desiredState;

    // Prevent rotating module if speed is less than 0.1%. Prevents Jittering.
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }

    state.optimize(new Rotation2d(getTurningEncoderRadians()));

    // Calculate the drive output from the drive PID controller.
    // Note: due to the drive PID constants being zero currently, this driveOutput will
    //       always be zero.
    final double driveOutput =
        m_drivePIDController.calculate(
            m_driveMotor.getVelocity().getValueAsDouble(), state.speedMetersPerSecond);

    final double driveFeedForward = state.speedMetersPerSecond / Constants.kMaxSpeedMetersPerSecond;

    // Calculate the turning motor output from the turning PID controller.
    final var turnOutput =
        m_turningPIDController.calculate(getTurningEncoderRadians(), state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(
        MathUtil.clamp(
            (driveOutput + driveFeedForward) * m_driveMotorGain,
            -1.0, // min -100%
            1.0) // max +100%
        );
    m_turningMotor.set(turnOutput);
  }
}
