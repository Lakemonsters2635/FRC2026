// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeAngleSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAngleUpCommand extends Command {
  /** Creates a new IntakeAngleCommand. */
  IntakeAngleSubsystem m_intakeAngleSubsystem;

  Timer m_timer;
  boolean feedForwardApplied;

  public IntakeAngleUpCommand(IntakeAngleSubsystem intakeAngleSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intakeAngleSubsystem = intakeAngleSubsystem;
    m_timer = new Timer();
    addRequirements(m_intakeAngleSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    // feedForwardApplied = false;
    m_intakeAngleSubsystem.intakeAngleUp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_intakeAngleSubsystem.intakeAngleDown();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeAngleSubsystem.intakeAngleFeedForward();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer.get() > 1) {
      return true;
    }
    return false;
  }
}
