// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeAngleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeOutCommand extends Command {
  IntakeSubsystem m_intakeSubsystem;
  IntakeAngleSubsystem m_intakeAngleSubsystem;

  public IntakeOutCommand(
      IntakeSubsystem m_intakeOutSubsystem, IntakeAngleSubsystem intakeAngleSubsystem) {
    m_intakeSubsystem = m_intakeOutSubsystem;
    m_intakeAngleSubsystem = intakeAngleSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeSubsystem);
    addRequirements(m_intakeAngleSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.intakeOut();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeAngleSubsystem.intakeBarelyUp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.intakeStop();
    m_intakeAngleSubsystem.intakeAngleStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
