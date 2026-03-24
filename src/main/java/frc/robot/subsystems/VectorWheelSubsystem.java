// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VectorWheelSubsystem extends SubsystemBase {
  /** Creates a new VectorWheelSubsystem. */
  TalonFX agitatorMotorTop;
  private VoltageConfigs m_voltageConfig= new VoltageConfigs();

  public VectorWheelSubsystem() {
    agitatorMotorTop = new TalonFX(Constants.AGITATOR_MOTOR_TOP_ID); // orange
    agitatorMotorTop.getConfigurator().apply(m_voltageConfig);
    SmartDashboard.putNumber("vectorPower", 0.5);
  }

  public void setVectorWheelsIn() {
    agitatorMotorTop.setVoltage(-1.5);
  }

  public void setVectorWheelsOut() {
    agitatorMotorTop.setVoltage(1.65);
  }

  public void stopVectorWheels() {
    agitatorMotorTop.setVoltage(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
