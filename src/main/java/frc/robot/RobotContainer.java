// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.print.attribute.standard.JobHoldUntil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AgitateCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeOutCommand;
import frc.robot.commands.ManualTurret;
import frc.robot.commands.PidAutoCommand;
import frc.robot.commands.SetLinearPoseCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.TransportCommand;
import frc.robot.commands.UptakeCommand;
import frc.robot.subsystems.ActuatorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {

  // Joysticks
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  public static Joystick rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_CHANNEL);
  public static Joystick leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_CHANNEL);

  DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  IntakeCommand m_intakeCommand = new IntakeCommand(m_intakeSubsystem);
  IntakeOutCommand m_intakeOutCommand = new IntakeOutCommand(m_intakeSubsystem);

  // ManualTurret m_manualTurret = new ManualTurret(m_turretSubsystem);

  public RobotContainer() {
    autoChooser.setDefaultOption(
        "Leave", new PidAutoCommand(m_drivetrainSubsystem, m_objectTrackerSubsystem, 0, 1, 0));
    autoChooser.addOption(
        "Climb", new PidAutoCommand(m_drivetrainSubsystem, m_objectTrackerSubsystem, 0, 1, 0));
    autoChooser.addOption(
        "Climb + Shoot",
        new PidAutoCommand(m_drivetrainSubsystem, m_objectTrackerSubsystem, 0, 1, 0));
    autoChooser.addOption(
        "Shoot", new PidAutoCommand(m_drivetrainSubsystem, m_objectTrackerSubsystem, 0, 1, 0));
    SmartDashboard.putData("Auto Mode", autoChooser);
    configureBindings();
  }

  // SUBSYSTEMS
  private static ActuatorSubsystem m_actuatorSubsystem = new ActuatorSubsystem();
  private static ObjectTrackerSubsystem m_objectTrackerSubsystem =
      new ObjectTrackerSubsystem("shripFront");
  private static ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private static TransportSubsystem m_transportSubsystem = new TransportSubsystem();
  private static TurretSubsystem m_turretSubsystem = new TurretSubsystem(m_objectTrackerSubsystem);

  // Cmmands
  private static TransportCommand m_transportCommand = new TransportCommand(m_transportSubsystem);
  private static AgitateCommand m_agitateCommand = new AgitateCommand(m_transportSubsystem);
  private static UptakeCommand m_uptakeCommand = new UptakeCommand(m_transportSubsystem);
  private static ShooterCommand m_shooterCommand = new ShooterCommand(m_shooterSubsystem);

  private void configureBindings() {
    Trigger intakeInward = new JoystickButton(leftJoystick, Constants.INTAKE_IN_BUTTON);
    Trigger intakeOutward = new JoystickButton(leftJoystick, Constants.INTAKE_OUT_BUTTON);
    // left joystick buttons
    Trigger shootButton = new JoystickButton(leftJoystick, 1);
    Trigger setHighButton = new JoystickButton(leftJoystick, 2);
    Trigger setLowButton = new JoystickButton(leftJoystick, 3);
    Trigger throttleControl = new JoystickButton(leftJoystick, 4);
    Trigger moveTurretLeft = new JoystickButton(leftJoystick, 5);
    Trigger moveTurretRight = new JoystickButton(leftJoystick, 6);
    
    // right joystick button
    Trigger manualTurredButton = new JoystickButton(rightJoystick, 4);
    
    // Untested
    Trigger aimTurretAtAprilTag = new JoystickButton(leftJoystick, 7);
    shootButton.whileTrue(m_shooterCommand);
    
    // manualTurredButton.onTrue(m_manualTurret);
    intakeInward.whileTrue(m_intakeCommand);
    intakeOutward.whileTrue(m_intakeOutCommand);

    setHighButton.onTrue(new SetLinearPoseCommand(m_actuatorSubsystem, 0.7));
    setLowButton.onTrue(new SetLinearPoseCommand(m_actuatorSubsystem, .3));
    throttleControl.whileTrue(
        new InstantCommand(
            () ->
                CommandScheduler.getInstance()
                    .schedule(
                        new SetLinearPoseCommand(
                            m_actuatorSubsystem,
                            MathUtil.clamp(Math.abs(leftJoystick.getThrottle()), .24, .8)))));

    // right joystick buttons
    Trigger transportButton = new JoystickButton(rightJoystick, 1);
    Trigger agitateButton = new JoystickButton(rightJoystick, 2);
    Trigger uptakeButton = new JoystickButton(rightJoystick, 3);

    // turretButton.whileTrue(new InstantCommand(()->m_turretSubsystem.turretPower(1)));
    // oppositeTurretButton.whileTrue(new InstantCommand(()->m_turretSubsystem.turretPower(-1)));

    transportButton.whileTrue(m_transportCommand);
    agitateButton.whileTrue(m_agitateCommand);
    uptakeButton.whileTrue(m_uptakeCommand);
    // moveTurretLeft.onTrue(new InstantCommand(() -> m_turretSubsystem.moveTurretLeft()));
    // moveTurretRight.onTrue(new InstantCommand(() -> m_turretSubsystem.moveTurretRight()));
    // Need to change this depending on the alliance, only for testing, be carefull
    // aimTurretAtAprilTag.onTrue(new InstantCommand(() -> m_turretSubsystem.aimAtTarget(10)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
