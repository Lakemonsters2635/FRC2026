// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeAngleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAutoCommand extends Command {
  /** Creates a new IntakeCommand. */
  IntakeSubsystem m_intakeSubsystem;

  IntakeAngleSubsystem m_intakeAngleSubsystem;

  public IntakeAutoCommand(IntakeSubsystem intakeSubsystem, IntakeAngleSubsystem intakeAngleSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    m_intakeAngleSubsystem = intakeAngleSubsystem;
    addRequirements(m_intakeSubsystem, intakeAngleSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeAngleSubsystem.chagePidMode(false);
    m_intakeAngleSubsystem.setVolts(2);
    m_intakeSubsystem.intakeIn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.intakeStop();
    m_intakeAngleSubsystem.setVolts(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
