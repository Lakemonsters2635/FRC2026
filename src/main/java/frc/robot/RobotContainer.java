// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SetLinearPoseCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.ActuatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

  // CONTROLLERS (eg. joystick, streamdeck, etc...)
  public static Joystick rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_PORT);
  public static Joystick leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_PORT);

  // SUBSYSTEMS
  private static ActuatorSubsystem m_actuatorSubsystem = new ActuatorSubsystem(); 
  private static ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private static ShooterCommand m_shooterCommand = new ShooterCommand(m_shooterSubsystem);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    Trigger shootButton = new JoystickButton(leftJoystick, 1);

    Trigger setHighButton = new JoystickButton(leftJoystick, 2);
    Trigger setLowButton = new JoystickButton(leftJoystick, 3);
    Trigger throttleControl = new JoystickButton(leftJoystick, 4);

    setHighButton.onTrue(new SetLinearPoseCommand(m_actuatorSubsystem, 0.7));
    setLowButton.onTrue(new SetLinearPoseCommand(m_actuatorSubsystem, .3));
    shootButton.whileTrue(m_shooterCommand);
    throttleControl.whileTrue(new InstantCommand(() ->
    CommandScheduler.getInstance().schedule(
    new SetLinearPoseCommand(m_actuatorSubsystem, MathUtil.clamp(Math.abs(leftJoystick.getThrottle()),.24,.8)
    ))));
  }
    public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
