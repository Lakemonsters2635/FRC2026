// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveTurretToPoseCommand extends Command {
  /** Creates a new MoveTurretToPoseCommand. */
  TurretSubsystem m_turretSubystem;
  int targetPose;
  public MoveTurretToPoseCommand(TurretSubsystem turretSubsystem, int target) {
    m_turretSubystem = turretSubsystem;
    targetPose = target;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_turretSubystem.getDegrees() > targetPose){
        m_turretSubystem.turretPower(Constants.TURRET_POWER);
    }
    else{

    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      if (Math.abs(m_turretSubystem.elevatorHeight() - targetPose) < Constants.TURRET_ANGLE_RANGE ) {
        return true;
      }

      return false;
    }
  }
}
