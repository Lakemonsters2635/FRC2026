// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  TalonFX m_shooterMotorLeft;

  TalonFX m_shooterMotorRight;
  Joystick joystick = new Joystick(0);
  double power = 0;

  public ShooterSubsystem() {
    m_shooterMotorLeft = new TalonFX(Constants.SHOOTER_MOTOR_ID_LEFT);
    m_shooterMotorRight = new TalonFX(Constants.SHOOTER_MOTOR_ID_RIGHT);
  }

  public void shoot() {
    m_shooterMotorRight.setVoltage(Constants.SHOOT * -1);
    m_shooterMotorLeft.setVoltage(Constants.SHOOT);
  }

  public void shooterStop() {
    m_shooterMotorLeft.setVoltage(Constants.MOTOR_STOP);
    m_shooterMotorRight.setVoltage(Constants.MOTOR_STOP);
  }

  @Override
  public void periodic() {
    // power = (joystick.getThrottle() + 1) * 5;
    // This method will be called once per scheduler run
  }
}
