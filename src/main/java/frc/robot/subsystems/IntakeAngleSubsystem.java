// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeAngleSubsystem extends SubsystemBase {
  TalonFX m_intakeAngleMotor;
  VoltageConfigs m_config;
  double m_targetPos;
  PIDController m_pidController = new PIDController(0.02, 0.001, 0);
  double ff = 0;
  double initialPos;
  boolean pidMode = false;

  public IntakeAngleSubsystem() {
    m_intakeAngleMotor = new TalonFX(Constants.INTAKE_ANGLE_MOTOR);
    m_intakeAngleMotor.setNeutralMode(NeutralModeValue.Brake);
    CurrentLimitsConfigs currentLimits =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(120)
            .withStatorCurrentLimitEnable(false)
            .withSupplyCurrentLimit(80)
            .withSupplyCurrentLimitEnable(false);

    m_config = new VoltageConfigs();
    m_config.SupplyVoltageTimeConstant = 10;
    m_intakeAngleMotor.getConfigurator().apply(m_config);
    m_intakeAngleMotor.getConfigurator().apply(currentLimits);

    initialPos = m_intakeAngleMotor.getPosition().getValueAsDouble();
    m_targetPos = getAngle();
  }

  public void chagePidMode(boolean newMode) {
    pidMode = newMode;
  }

  public double getAngle() {
    // Range should be 0 to 90 apprx
    return -1
        * (m_intakeAngleMotor.getPosition().getValueAsDouble() - initialPos)
        / (6.578128)
        * 360;
  }

  public double getRawEncoderPos(){
    return m_intakeAngleMotor.getPosition().getValueAsDouble();
  }

  public double getCommandedVoltage(){
    return m_intakeAngleMotor.getMotorVoltage().getValueAsDouble();
  }
  public void resetEncoder() {
    m_intakeAngleMotor.setPosition(0);
  }

  public void setInitialPos() {
    initialPos = m_intakeAngleMotor.getPosition().getValueAsDouble();
  }

  public void intakeAngleDown() {
    // not tested
    m_intakeAngleMotor.setVoltage(0.5); // +
  }

  public void intakeBarelyUp() {
    m_intakeAngleMotor.setVoltage(-1.50);
  }

  public void intakeAngleUp() {
    // not tested
    m_intakeAngleMotor.setVoltage(-4); // +
  }

  public void intakeAngleDownHard() {
    // not tested
    m_intakeAngleMotor.setVoltage(1); // +
  }

  public void setVolts(double volts) {
    m_intakeAngleMotor.setVoltage(volts);
  }

  public void intakeAngleFeedForward() {
    m_intakeAngleMotor.setVoltage(-.25); // working, but change for better results
  }

  public void intakeAngleStop() {
    m_intakeAngleMotor.setVoltage(0); // -
  }

  public void setTargetPos(double target) {
    m_targetPos = target;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("InAngle: curr angle", getAngle());
    SmartDashboard.putNumber("InAngle initialPos", initialPos);
    SmartDashboard.putNumber("InAngle Commanded Voltage", m_intakeAngleMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber(
        "InAngle raw encoder", m_intakeAngleMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber(
        "Falcon (Intake Arm) Temp", m_intakeAngleMotor.getDeviceTemp().getValueAsDouble());

    if (pidMode) {
      double fb = m_pidController.calculate(getAngle(), m_targetPos);
      SmartDashboard.putNumber("inAngle fb", fb);
      ff = -2.3; // -2.36;
      // setVolts(ff * Math.cos(getAngle() * Math.PI/180));
      setVolts(ff * Math.abs(Math.cos(getAngle() * Math.PI / 180)) - fb);
    }
  }
}
