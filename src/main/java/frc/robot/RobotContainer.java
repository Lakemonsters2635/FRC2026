// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeOutCommand;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {

  Joystick leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_ID);
  Joystick rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_ID);

  IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  IntakeCommand m_intakeCommand = new IntakeCommand(m_intakeSubsystem);
  IntakeOutCommand m_intakeOutCommand = new IntakeOutCommand(m_intakeSubsystem);
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    Trigger intakeInward = new JoystickButton(leftJoystick, Constants.INTAKE_IN_BUTTON_NUMBER);
    Trigger intakeOutward = new JoystickButton(leftJoystick,Constants.INTAKE_OUT_BUTTON_NUMBER);
    intakeInward.whileTrue(m_intakeCommand);
    intakeOutward.whileTrue(m_intakeOutCommand);
  }
  
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
