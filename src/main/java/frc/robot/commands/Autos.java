// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Autos extends Command {

  DrivetrainSubsystem m_dts;
  ObjectTrackerSubsystem m_objectTrackerSubsystem;
  TransportSubsystem m_transportSubsystem;
  ShooterSubsystem m_shooterSubsystem;
  TurretSubsystem m_turretSubsystem;

  /** Creates a new Autos. */
  public Autos(
      DrivetrainSubsystem dts,
      ObjectTrackerSubsystem objectTrackerSubsystem,
      TransportSubsystem transportSubsystem,
      ShooterSubsystem shooterSubsystem,
      TurretSubsystem turretSubsystem) {
    m_dts = dts;
    m_objectTrackerSubsystem = objectTrackerSubsystem;
    m_transportSubsystem = transportSubsystem;
    m_shooterSubsystem = shooterSubsystem;
    m_turretSubsystem = turretSubsystem;
  }

  public Command goStraight() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> m_dts.stopMotors()).withTimeout(0.1),
        new InstantCommand(() -> m_dts.setFollowJoystick(false)).withTimeout(0.1),
        m_dts.createPath(
            new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(-90))),
            new Translation2d(0, 0.5),
            new Pose2d(0, 1, new Rotation2d(Units.degreesToRadians(-90)))),
        new InstantCommand(() -> m_dts.setFollowJoystick(true)).withTimeout(0.1),
        new InstantCommand(() -> m_dts.stopMotors()),
        new InstantCommand(() -> m_dts.resetAngle(180)),
        new InstantCommand(() -> m_dts.zeroOdometry()));
  }

  public Command middleShootAuto() {
    return new SequentialCommandGroup(
        new AgitateCommand(m_transportSubsystem),
        new WaitCommand(0.5),
        new UptakeCommand(m_transportSubsystem),
        new WaitCommand(0.5),
        new ShooterCommand(m_shooterSubsystem));
  }

  public Command leftShootAuto() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> m_dts.stopMotors()).withTimeout(0.1),
        new InstantCommand(() -> m_dts.setFollowJoystick(false)).withTimeout(0.1),
        m_dts.createPath(
            new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(-90))),
            new Translation2d(0, 0.99),
            new Pose2d(0, 1.98, new Rotation2d(Units.degreesToRadians(-90)))),
        new InstantCommand(() -> m_dts.setFollowJoystick(true)).withTimeout(0.1),
        new InstantCommand(() -> m_dts.stopMotors()),
        new InstantCommand(() -> m_dts.resetAngle(180)),
        new InstantCommand(() -> m_dts.zeroOdometry()),
      new AgitateCommand(m_transportSubsystem),
      new WaitCommand(0.5),
      new UptakeCommand(m_transportSubsystem),
      new WaitCommand(0.5),
      new MoveTurretToPoseCommand(m_turretSubsystem, 45), //TODO check angle
      new WaitCommand(0.5),
      new ShooterCommand(m_shooterSubsystem)

    );
  }

  public Command rightShootAuto() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> m_dts.stopMotors()).withTimeout(0.1),
        new InstantCommand(() -> m_dts.setFollowJoystick(false)).withTimeout(0.1),
        m_dts.createPath(
            new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(-90))),
            new Translation2d(0, 0.99),
            new Pose2d(0, 1.98, new Rotation2d(Units.degreesToRadians(-90)))),
        new InstantCommand(() -> m_dts.setFollowJoystick(true)).withTimeout(0.1),
        new InstantCommand(() -> m_dts.stopMotors()),
        new InstantCommand(() -> m_dts.resetAngle(180)),
        new InstantCommand(() -> m_dts.zeroOdometry()),
      new AgitateCommand(m_transportSubsystem),
      new WaitCommand(0.5),
      new UptakeCommand(m_transportSubsystem),
      new WaitCommand(0.5),
      new MoveTurretToPoseCommand(m_turretSubsystem, -45), //TODO check angle
      new WaitCommand(0.5),
      new ShooterCommand(m_shooterSubsystem)

    );
  }





//---------



  public Command straightScoreAuto() {
    return new SequentialCommandGroup(
        new VisionAutoCommand(m_dts, m_objectTrackerSubsystem, 10, 6, -58.5, 0, -90, true));
  }

  public Command leftScoreAuto() {
    return new SequentialCommandGroup(
        new PidAutoCommand(
            m_dts, m_objectTrackerSubsystem, 0, -Units.inchesToMeters(148.375 - 11 - 12), 0),
        new WaitCommand(0.3),
        new VisionAutoCommand(m_dts, m_objectTrackerSubsystem, 2, 6, -58.5 + 2, 0, -90, true));
  }

  public Command rightScoreAuto() {
    return new SequentialCommandGroup(
        new PidAutoCommand(
            m_dts, m_objectTrackerSubsystem, 0, Units.inchesToMeters(148.375 - 11 - 12), 0),
        new WaitCommand(0.3),
        new VisionAutoCommand(m_dts, m_objectTrackerSubsystem, 5, 6, -58.5 + 2, 0, -90, true));
  }
}
