// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  TalonFX m_intakeMotor;
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_ID);
    SmartDashboard.putNumber("intake voltage", 0);


  }

  public void intakeIn() {
    m_intakeMotor.setVoltage((SmartDashboard.getNumber("intake voltage", 0)));

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
