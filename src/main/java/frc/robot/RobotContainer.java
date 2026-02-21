// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AgitateCommand;
import frc.robot.commands.TransportCommand;
import frc.robot.commands.UptakeCommand;
import frc.robot.subsystems.TransportSubsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  public static Joystick rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_ID);
  public static Joystick leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_ID);

  public static TransportSubsystem m_transportSubsystem = new TransportSubsystem();
  
  public static TransportCommand m_transportCommand = new TransportCommand(m_transportSubsystem);
  public static AgitateCommand m_agitateCommand = new AgitateCommand(m_transportSubsystem);
  public static UptakeCommand m_uptakeCommand = new UptakeCommand(m_transportSubsystem);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    Trigger transportButton = new JoystickButton(leftJoystick, 1);
    Trigger agitateButton = new JoystickButton(leftJoystick, 2);
    Trigger uptakeButton = new JoystickButton(leftJoystick, 3);

    transportButton.whileTrue(m_transportCommand);
    agitateButton.whileTrue(m_agitateCommand);  
    uptakeButton.whileTrue(m_uptakeCommand);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
