// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSybsystem. */
  TalonFX shooterMotor;
  public ShooterSubsystem() {
    shooterMotor = new TalonFX(Constants.SHOOTER_MOTOR_ID);
  }

  public void shoot(){
    shooterMotor.setVoltage(Constants.SHOOT_OUT_ID);
  }

  public void shooterStop(){
    shooterMotor.setVoltage(Constants.MOTOR_STOP_ID);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
