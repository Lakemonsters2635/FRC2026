// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SetLinearPoseCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.ActuatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AgitateCommand;
import frc.robot.commands.TransportCommand;
import frc.robot.commands.UptakeCommand;
import frc.robot.commands.VisionAutoCommand;
import frc.robot.subsystems.TransportSubsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.DrivetrainSubsystem;

public class RobotContainer {

  // Joysticks
  public static Joystick rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_CHANNEL);
  public static Joystick leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_CHANNEL);

  DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  public RobotContainer() {
    configureBindings();
  }

  // SUBSYSTEMS
  private static ActuatorSubsystem m_actuatorSubsystem = new ActuatorSubsystem(); 
  private static ObjectTrackerSubsystem m_objectTrackerSubsystem = new ObjectTrackerSubsystem("front");
  private static ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private static TransportSubsystem m_transportSubsystem = new TransportSubsystem();
  private static TurretSubsystem m_turretSubsystem = new TurretSubsystem(m_objectTrackerSubsystem);
  
   //Cmmands
  private static TransportCommand m_transportCommand = new TransportCommand(m_transportSubsystem);
  private static AgitateCommand m_agitateCommand = new AgitateCommand(m_transportSubsystem);
  private static UptakeCommand m_uptakeCommand = new UptakeCommand(m_transportSubsystem);
  private static ShooterCommand m_shooterCommand = new ShooterCommand(m_shooterSubsystem);
 

  private void configureBindings() {

    //left joystick buttons
    Trigger shootButton = new JoystickButton(leftJoystick, 1);
    Trigger setHighButton = new JoystickButton(leftJoystick, 2);
    Trigger setLowButton = new JoystickButton(leftJoystick, 3);
    Trigger throttleControl = new JoystickButton(leftJoystick, 4);
    Trigger moveTurretLeft = new JoystickButton(leftJoystick, 5);
    Trigger moveTurretRight = new JoystickButton(leftJoystick, 6);
    shootButton.whileTrue(m_shooterCommand);

    setHighButton.onTrue(new SetLinearPoseCommand(m_actuatorSubsystem, 0.7));
    setLowButton.onTrue(new SetLinearPoseCommand(m_actuatorSubsystem, .3));
    throttleControl.whileTrue(new InstantCommand(() -> 
    CommandScheduler.getInstance().schedule(
    new SetLinearPoseCommand(m_actuatorSubsystem, MathUtil.clamp(Math.abs(leftJoystick.getThrottle()),.24,.8)
    ))));


    
    
    //right joystick buttons
    Trigger transportButton = new JoystickButton(rightJoystick, 1);
    Trigger agitateButton = new JoystickButton(rightJoystick, 2);
    Trigger uptakeButton = new JoystickButton(rightJoystick, 3);

    // turretButton.whileTrue(new InstantCommand(()->m_turretSubsystem.turretPower(1)));
    // oppositeTurretButton.whileTrue(new InstantCommand(()->m_turretSubsystem.turretPower(-1)));

    transportButton.whileTrue(m_transportCommand);
    agitateButton.whileTrue(m_agitateCommand);  
    uptakeButton.whileTrue(m_uptakeCommand);
    moveTurretLeft.onTrue(new InstantCommand(()->m_turretSubsystem.moveTurretLeft()));
    moveTurretRight.onTrue(new InstantCommand(()->m_turretSubsystem.moveTurretRight()));

  }
    public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}