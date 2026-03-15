// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeAngleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.UptakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class UptakeReverseCommand extends Command {
  /** Creates a new UptakeCommand. */
  UptakeSubsystem m_uptakeSubsystem;

  public UptakeReverseCommand(UptakeSubsystem uptakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_uptakeSubsystem = uptakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_uptakeSubsystem.reverseUptake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_uptakeSubsystem.stopUptake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
