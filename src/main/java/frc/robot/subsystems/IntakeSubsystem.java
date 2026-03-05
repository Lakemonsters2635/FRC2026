// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  TalonFX intakeMotor;
  private final SparkMax m_intakeMotor;
  private final SparkMaxConfig m_intakeConfig;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_intakeMotor = new SparkMax(Constants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    m_intakeConfig = new SparkMaxConfig();

    m_intakeMotor.configure(m_intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void intakeIn() {
    m_intakeMotor.setVoltage(Constants.IN_VOLTAGE);
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
