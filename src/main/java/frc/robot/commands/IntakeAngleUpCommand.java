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

  double initAngle = 0;
  Timer m_timer;
  boolean feedForwardApplied;

  public IntakeAngleUpCommand(IntakeAngleSubsystem intakeAngleSubsystem) {
    m_intakeAngleSubsystem = intakeAngleSubsystem;
    addRequirements(m_intakeAngleSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_intakeAngleSubsystem.getRawEncoderPos() > -0.65){
      m_intakeAngleSubsystem.setInitialPos();
      initAngle = m_intakeAngleSubsystem.getAngle();

    // feedForwardApplied = false;
      m_intakeAngleSubsystem.chagePidMode(true);
      m_intakeAngleSubsystem.setTargetPos(120);
    }
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (initAngle + 90 < m_intakeAngleSubsystem.getAngle()) {
      m_intakeAngleSubsystem.setVolts(-1);
    }
    // m_intakeAngleSubsystem.setVolts(m_intakeAngleSubsystem.getCommandedVoltage()-0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
