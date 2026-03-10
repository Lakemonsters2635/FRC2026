// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  ObjectTrackerSubsystem m_objectTrackerSubsystem;
  TalonFX m_shooterMotorLeft;

  TalonFX m_shooterMotorRight;
  Joystick joystick = new Joystick(0);
  double power = 7;

  public ShooterSubsystem(ObjectTrackerSubsystem objectTrackerSubsystem) {
    m_objectTrackerSubsystem = objectTrackerSubsystem;
    m_shooterMotorLeft = new TalonFX(Constants.SHOOTER_MOTOR_ID_LEFT);
    m_shooterMotorRight = new TalonFX(Constants.SHOOTER_MOTOR_ID_RIGHT);
    SmartDashboard.putNumber("Shooter Power", 8);
  }

  public void shoot() {
    Pose2d target = m_objectTrackerSubsystem.getDistVector(0, Units.metersToInches(.6), 0, 10);
    double magnitude = Math.sqrt(target.getX() * target.getX() + target.getY() * target.getY());
    m_shooterMotorRight.setVoltage(((power)+(magnitude/4.5))*-1);//SmartDashboard.getNumber("Shooter Power", 8) * -1);
    m_shooterMotorLeft.setVoltage((power+(magnitude/4.5)));SmartDashboard.getNumber("Shooter Power", 8);
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
