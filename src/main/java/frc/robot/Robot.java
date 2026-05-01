// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private GenericEntry batteryEntry;

  private final RobotContainer m_robotContainer;
  private final PowerDistribution pdh;

  public Robot() {
    m_robotContainer = new RobotContainer();
    pdh = new PowerDistribution();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    double volts = pdh.getVoltage();
    double time = DriverStation.getMatchTime();
    // Shuffleboard.getTab("Driver")
    //   .add("Battery Voltage", volts)
    //   .withWidget(BuiltInWidgets.kVoltageView)
    //   .getEntry();
    SmartDashboard.putNumber("Battery Voltage", volts);
    SmartDashboard.putNumber("Match Time", time);
    if (volts < 8.0) {
      SmartDashboard.putBoolean("Battery Status", false);
    } else {
      SmartDashboard.putBoolean("Battery Status", true);
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_robotContainer.m_drivetrainSubsystem.setFollowJoystick(false);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    m_robotContainer.m_drivetrainSubsystem.setFollowJoystick(true);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
