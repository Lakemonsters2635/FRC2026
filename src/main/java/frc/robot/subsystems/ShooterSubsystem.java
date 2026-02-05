// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  TalonFX m_shooterMotor;
  public ShooterSubsystem() {
    m_shooterMotor = new TalonFX(Constants.SHOOTER_MOTOR_ID);
  }

  public void shoot(){
    m_shooterMotor.setVoltage(Constants.SHOOT);
  }

  public void shooterStop(){
    m_shooterMotor.setVoltage(Constants.MOTOR_STOP);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
