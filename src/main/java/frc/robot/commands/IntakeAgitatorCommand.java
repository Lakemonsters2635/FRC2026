// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeAngleSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAgitatorCommand extends Command {
  /** Creates a new IntakeAgitatorCommand. */
  IntakeAngleSubsystem m_ias;

  public IntakeAgitatorCommand(IntakeAngleSubsystem m_intakeAngleSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ias = m_intakeAngleSubsystem;
    addRequirements(m_ias);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_ias.chagePidMode(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double timeMS = Math.abs(RobotContainer.rightJoystick.getThrottle()*2000);
    double timeMS = 330;
    SmartDashboard.putNumber("timeMS", timeMS);
    if (System.currentTimeMillis() % timeMS < timeMS / 2) {
      // m_ias.setTargetPos(30);
      m_ias.setVolts(-3.5);
    } else {
      // m_ias.setTargetPos(0);
      m_ias.setVolts(0.4);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_ias.chagePidMode(false);
    m_ias.intakeAngleStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
