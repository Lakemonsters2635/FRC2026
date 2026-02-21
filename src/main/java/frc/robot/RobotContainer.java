// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {

  // CONTROLLERS (eg. joystick, streamdeck, etc...)
  public static Joystick rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_PORT);
  public static Joystick leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_PORT);

  // SUBSYSTEMS
  public static DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  public static TurretSubsystem m_turretSubsystem = new TurretSubsystem();


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    Trigger turretButton = new JoystickButton(rightJoystick, 1);
    turretButton.whileTrue(new InstantCommand(()->m_turretSubsystem.turretPower(1)));
    Trigger oppositeTurretButton = new JoystickButton(rightJoystick, 2);
    oppositeTurretButton.whileTrue(new InstantCommand(()->m_turretSubsystem.turretPower(-1)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
