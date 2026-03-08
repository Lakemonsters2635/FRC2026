// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RollerSubsystem extends SubsystemBase {
  /** Creates a new RollerSubsystem. */
  TalonFX agitatorMotorBottom;

  public RollerSubsystem() {
    agitatorMotorBottom = new TalonFX(Constants.AGITATOR_MOTOR_BOTTOM_ID); // white
    SmartDashboard.putNumber("rollerPower", 2);
  }

  public void setRollersBackward() {
    agitatorMotorBottom.setVoltage(-SmartDashboard.getNumber("rollerPower", 2));
  }

  public void setRollersForward() {
    agitatorMotorBottom.setVoltage(SmartDashboard.getNumber("rollerPower", 2));
  }

  public void stopRollers() {
    agitatorMotorBottom.setVoltage(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
