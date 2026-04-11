// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class IntakeSubsystem extends SubsystemBase {
  TalonFX m_intakeMotor;
  boolean highPower = false;
  private VoltageConfigs m_voltageConfig = new VoltageConfigs();

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_voltageConfig.SupplyVoltageTimeConstant = 10;
    m_intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_ID);
    m_intakeMotor.getConfigurator().apply(m_voltageConfig);
  }

  public void intakeIn() {
    // if (highPower) {
      m_intakeMotor.setVoltage(12);
    // } else {
    //   m_intakeMotor.setVoltage(6);
    // }
  }

  public void setBoolHighPower(boolean isHighPower) {
    highPower = isHighPower;
    SmartDashboard.putNumber("throttle val", RobotContainer.leftJoystick.getThrottle() * 3);
  }

  public void intakeOut() {
    m_intakeMotor.setVoltage(Constants.OUT_VOLTAGE);
  }

  public void intakeStop() {
    m_intakeMotor.setVoltage(Constants.MOTOR_STOP);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
