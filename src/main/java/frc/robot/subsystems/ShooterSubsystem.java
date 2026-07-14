// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  ObjectTrackerSubsystem m_objectTrackerSubsystem;

  boolean isMidMode = false;
  TalonFX m_shooterMotorLeft;

  TalonFX m_shooterMotorRight;
  VoltageConfigs m_leftConfig;
  VoltageConfigs m_rightConfig;

  TalonFX motor = new TalonFX(1);
  VelocityVoltage velocityRequest = new VelocityVoltage(0);
  Slot0Configs slot0Configs = new Slot0Configs();
  CurrentLimitsConfigs currentLimits =
      new CurrentLimitsConfigs()
          .withStatorCurrentLimit(60)
          .withStatorCurrentLimitEnable(true)
          .withSupplyCurrentLimit(60)
          .withSupplyCurrentLimitEnable(true);

  Joystick joystick = new Joystick(0);
  double power = 7;
  double savePower;
  double magnitude;
  double desiredVelocity = 60;
  double joystickDeltaPower = 0;

  public ShooterSubsystem(ObjectTrackerSubsystem objectTrackerSubsystem) {
    CurrentLimitsConfigs currentLimits =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(60)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(60)
            .withSupplyCurrentLimitEnable(true);

    m_leftConfig = new VoltageConfigs();
    m_rightConfig = new VoltageConfigs();
    m_rightConfig.SupplyVoltageTimeConstant = 10;
    m_leftConfig.SupplyVoltageTimeConstant = 10;

    m_objectTrackerSubsystem = objectTrackerSubsystem;
    m_shooterMotorLeft = new TalonFX(Constants.SHOOTER_MOTOR_ID_LEFT);
    m_shooterMotorRight = new TalonFX(Constants.SHOOTER_MOTOR_ID_RIGHT);

    m_shooterMotorLeft.getConfigurator().apply(m_leftConfig);
    m_shooterMotorLeft.getConfigurator().apply(currentLimits);
    m_shooterMotorRight.getConfigurator().apply(m_rightConfig);
    m_shooterMotorRight.getConfigurator().apply(currentLimits);

    slot0Configs.kG = 0;
    SmartDashboard.putNumber("Shooter P", 0.2);
    SmartDashboard.putNumber("Shooter I", 0);
    SmartDashboard.putNumber("Shooter D", 0);
    SmartDashboard.putNumber("Shooter S", 0);
    SmartDashboard.putNumber("Shooter V", 0.115);

    SmartDashboard.putNumber("velocity shooter rps", 1);
    // setPIDSV(SmartDashboard.getNumber("Shooter P", 0.2), SmartDashboard.getNumber("Shooter I",
    // 0), SmartDashboard.getNumber("Shooter D", 0), SmartDashboard.getNumber("Shooter S", 0),
    // SmartDashboard.getNumber("Shooter V", 0.115));
    setPIDSV(
        0.4,
        SmartDashboard.getNumber("Shooter I", 0),
        SmartDashboard.getNumber("Shooter D", 0),
        SmartDashboard.getNumber("Shooter S", 0),
        SmartDashboard.getNumber("Shooter V", 0.115));

    // SmartDashboard.putNumber("Shooter Power", 8);
  }

  public void deltaShooterVoltage(double deltaPower) {
    joystickDeltaPower += deltaPower;
  }

  public void setDeltaShooterVoltage(double voltage) {
    joystickDeltaPower = voltage;
  }

  public void shoot() {
    Pose2d target = m_objectTrackerSubsystem.getNearestAprilTagDistTurret();

    magnitude = Math.sqrt(target.getX() * target.getX() + target.getY() * target.getY());
    if (magnitude != 0) {
      // desiredVelocity = -0.658669 * magnitude * magnitude * magnitude + 6.08625 * magnitude *
      // magnitude -13.711 * magnitude + 65.43 + 1;
      desiredVelocity =
          (0.139446 * magnitude * magnitude * magnitude
            - 2.62406 * magnitude * magnitude
            + 16.19286 * magnitude
            + 36.23951)
          * 0.75;
    }
    // desiredVelocity = SmartDashboard.getNumber("velocity shooter rps", 1);
    velocityController(desiredVelocity);
  }

  public void setPIDSV(double p, double i, double d, double s, double v) {
    slot0Configs.kP = p;
    slot0Configs.kI = i;
    slot0Configs.kD = d;
    slot0Configs.kS = s;
    slot0Configs.kV = v;

    m_shooterMotorLeft.getConfigurator().apply(slot0Configs);
    m_shooterMotorRight.getConfigurator().apply(slot0Configs);
  }

  public void velocityController(double velocity) { // in revolutions/sec
    double tempVelocity = velocity; // SmartDashboard.getNumber("velocity shooter rps", 1);
    m_shooterMotorLeft.setControl(velocityRequest.withVelocity(tempVelocity));
    m_shooterMotorRight.setControl(velocityRequest.withVelocity(-tempVelocity));
  }

  public void shootFar() {
    m_shooterMotorRight.setVoltage(
        Constants.SHOOTER_FAR_POWER * -1); // SmartDashboard.getNumber("Shooter Power", 8) * -1);
    m_shooterMotorLeft.setVoltage(
        Constants.SHOOTER_FAR_POWER); // SmartDashboard.getNumber("Shooter Power", 8);
    // SmartDashboard.putNumber("Shooter Volt", (power + (magnitude / 4.35)));
  }

  public void shooterStop() {
    m_shooterMotorLeft.setVoltage(Constants.MOTOR_STOP);
    m_shooterMotorRight.setVoltage(Constants.MOTOR_STOP);
  }

  @Override
  public void periodic() {
    // shoot();
    // setPIDSV(0.4, SmartDashboard.getNumber("Shooter I", 0), SmartDashboard.getNumber("Shooter D",
    // 0), SmartDashboard.getNumber("Shooter S", 0), SmartDashboard.getNumber("Shooter V", 0.115));

    // velocityController(SmartDashboard.getNumber("velocity shooter rps", 0));
    SmartDashboard.getNumber("Shooter I", 0);
    SmartDashboard.getNumber("Shooter D", 0);
    SmartDashboard.getNumber("Shooter V", 0.115);
    SmartDashboard.getNumber("velocity shooter rps", 1);

    SmartDashboard.putNumber(
        "getShooterVelocityLeft", m_shooterMotorLeft.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber(
        "getShooterVelocityRight", m_shooterMotorLeft.getVelocity().getValueAsDouble());

    SmartDashboard.putNumber("mag of dist center camera to center hub", magnitude);

    // power = (joystick.getThrottle() + 1) * 5;
    // This method will be called once per scheduler run
  }
}
