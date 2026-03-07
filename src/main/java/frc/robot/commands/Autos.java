// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Autos extends Command {

  DrivetrainSubsystem m_dts;
  ObjectTrackerSubsystem m_objectTrackerSubsystem;
  ShooterSubsystem m_shooterSubsystem;

  /** Creates a new Autos. */
  public Autos(DrivetrainSubsystem dts, ObjectTrackerSubsystem objectTrackerSubsystem, ShooterSubsystem shooterSubsystem) {
    m_dts = dts;
    m_objectTrackerSubsystem = objectTrackerSubsystem;
    m_shooterSubsystem = shooterSubsystem;
  }

  public Command autoChoose() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (!alliance.isPresent()) {
      return goStraight();
    }

    // Logic to get the alliance number and auto choose the auto
    return new Command() {
      
    };
    }

  public Command goStraight() {
    return new SequentialCommandGroup(
        new PidAutoCommand(m_dts, m_objectTrackerSubsystem, 0, 1, 0)
      );
  }

  public Command straightScoreAuto() {
    return new SequentialCommandGroup(
        new VisionAutoCommand(m_dts, m_objectTrackerSubsystem, 10, 6, -58.5, 0, -90, true)
        );
  }

  public Command leftScoreAuto() {
    return new SequentialCommandGroup(
        new PidAutoCommand(
            m_dts, m_objectTrackerSubsystem, 0, -Units.inchesToMeters(148.375 - 11 - 12), 0),
        new WaitCommand(0.3),
        new VisionAutoCommand(m_dts, m_objectTrackerSubsystem, 2, 6, -58.5 + 2, 30, -90, true));
  }

  public Command rightScoreAuto() {
    return new SequentialCommandGroup(
        new PidAutoCommand(
            m_dts, m_objectTrackerSubsystem, 0, Units.inchesToMeters(148.375 - 11 - 12), 0),
        new WaitCommand(0.3),
        new VisionAutoCommand(m_dts, m_objectTrackerSubsystem, 5, 6, -58.5 + 2, -30, -90, true));
  }
}
