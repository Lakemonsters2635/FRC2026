// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeAngleSubsystem extends SubsystemBase {
  /** Creates a new IntakeAngleSubsystem. */
  TalonFX intakeAngleMotor;

  public IntakeAngleSubsystem() {
    intakeAngleMotor = new TalonFX(11);
    SmartDashboard.putNumber("Intake Angle voltage", 0);
  }

  public void intakeAngleDown() {
    // not tested
    intakeAngleMotor.setVoltage(3); // +
  }

  public void setVolts(double volts) {
    intakeAngleMotor.setVoltage(volts);
  }

  public void intakeAngleStop() {
    intakeAngleMotor.setVoltage(-3); // -
  }

  @Override
  public void periodic() {}
}
