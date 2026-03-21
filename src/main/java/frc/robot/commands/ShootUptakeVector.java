// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ActuatorSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.UptakeSubsystem;
import frc.robot.subsystems.VectorWheelSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootUptakeVector extends Command {
  /** Creates a new Shoot. */
  UptakeSubsystem m_us;

  RollerSubsystem m_rs;
  ShooterSubsystem m_ss;
  VectorWheelSubsystem m_vws;
  ActuatorSubsystem m_as;
  boolean m_aimingAprilTag; // If we are aiming at the apriltag(auto angle ajust)

  public ShootUptakeVector(
      UptakeSubsystem us,
      RollerSubsystem rs,
      ShooterSubsystem ss,
      VectorWheelSubsystem vws, 
      ActuatorSubsystem as,
      Boolean aimingAprilTag) { // this is for if we are shooting far or close
    m_us = us;
    m_rs = rs;
    m_ss = ss;
    m_vws = vws;
    m_as = as;
    m_aimingAprilTag = aimingAprilTag;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("m_aimingAtAprilTag", m_aimingAprilTag);
    // Add back later
    // if (m_aimingAprilTag) {
    //   m_ss.shoot();
    //   //m_ss.velocityController(0);
    // } else {
    //   // m_ss.shootFar();
    //   // m_as.setPosition(0.6);
    // }
    m_ss.shoot();
    m_us.uptake();
    m_vws.setVectorWheelsIn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (System.currentTimeMillis() % (Constants.SHAKE_INTERVAL_MS * 1.5)
        > Constants.SHAKE_INTERVAL_MS) {
      m_rs.setRollersForward();
    } else {
      //m_rs.setRollersBackward();
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
