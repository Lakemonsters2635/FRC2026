// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TransportSubsystem extends SubsystemBase {
  /** Creates a new TransportSubsystem. */
  TalonFX agitatorMotorTop;

  TalonFX agitatorMotorBottom;
  TalonFX uptakeMotor;

  public TransportSubsystem() {
    agitatorMotorTop = new TalonFX(Constants.AGITATOR_MOTOR_TOP_ID); // orange
    agitatorMotorBottom = new TalonFX(Constants.AGITATOR_MOTOR_BOTTOM_ID); // white

    uptakeMotor = new TalonFX(Constants.UPTAKE_MOTOR_ID);
  }

  public void agitate() {
    // not tested
    agitatorMotorTop.setVoltage(-0.5);
    agitatorMotorBottom.setVoltage(0.5);
  }

  public void uptake() {
    // not tested
    uptakeMotor.setVoltage(-3);
  }

  public void stopAgitate() {
    agitatorMotorTop.setVoltage(0);
    agitatorMotorBottom.setVoltage(0);
  }

  public void stopUptake() {
    uptakeMotor.setVoltage(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
