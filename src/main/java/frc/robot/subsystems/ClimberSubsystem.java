// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  TalonFX climberMotor;

  SparkMax hookMotor;
  SparkMaxConfig hookMotorConfig;

  @SuppressWarnings("removal")
  public ClimberSubsystem() {
    climberMotor = new TalonFX(Constants.CLIMBER_MOTOR_ID);
    hookMotor = new SparkMax(Constants.HOOK_MOTOR_ID, MotorType.kBrushless);
    hookMotorConfig = new SparkMaxConfig();
    hookMotorConfig.idleMode(IdleMode.kBrake);
    hookMotorConfig.inverted(false); // TODO: test and adjust this

    hookMotor.configure(
        hookMotorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  public void setHookSpeed(double speed) {
    hookMotor.setVoltage(speed * 12);
  }

  public void setClimberSpeed(double speed) {
    hookMotor.setVoltage(speed * 12);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
