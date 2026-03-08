// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.UptakeSubsystem;
import frc.robot.subsystems.VectorWheelSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
  /** Creates a new Shoot. */
  UptakeSubsystem m_us;

  RollerSubsystem m_rs;
  ShooterSubsystem m_ss;
  VectorWheelSubsystem m_vws;

  public Shoot(
      UptakeSubsystem us, RollerSubsystem rs, ShooterSubsystem ss, VectorWheelSubsystem vws) {
    m_us = us;
    m_rs = rs;
    m_ss = ss;
    m_vws = vws;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ss.shoot();
    m_us.uptake();
    m_vws.setVectorWheelsIn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (System.currentTimeMillis() % (Constants.SHAKE_INTERVAL_MS * 2)
        > Constants.SHAKE_INTERVAL_MS) {
      m_rs.setRollersForward();
    } else {
      m_rs.setRollersBackward();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ss.shooterStop();
    m_us.stopUptake();
    m_rs.stopRollers();
    m_vws.stopVectorWheels();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
