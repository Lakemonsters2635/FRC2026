// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

  // CONTROLLERS (eg. joystick, streamdeck, etc...)
  public static Joystick rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_PORT);
  public static Joystick leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_PORT);

  // SUBSYSTEMS
  private static ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private static ShooterCommand m_shooterCommand = new ShooterCommand(m_shooterSubsystem);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    Trigger shootButton = new JoystickButton(leftJoystick, 1);
    shootButton.whileTrue(m_shooterCommand);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
