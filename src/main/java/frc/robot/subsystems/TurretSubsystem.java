// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.VisionAutoCommand;
import java.util.Optional;

/*
 * This subsystem is intended to rotate the turret horizontally
 */
public class TurretSubsystem extends SubsystemBase {
  /** */
  ObjectTrackerSubsystem m_objectTrackerSubsystem;

  VisionAutoCommand m_visionAutoCommand;
  private final SparkMax m_turretSparkMax;
  private final SparkMaxConfig m_turretConfig;
  PIDController m_turretController;
  double pid;
  public double m_poseTarget;
  private boolean isAutoControl = true;
  public double aprilTagX = 0;
  public double aprilTagY = 0;
  public double aprilTagDelataRot = 0;
  public boolean canSeeAprilTag = false;

  public TurretSubsystem(ObjectTrackerSubsystem objectTrackerSubsystem) {
    m_turretSparkMax = new SparkMax(Constants.TURRET_MOTOR_ID, MotorType.kBrushless);
    m_turretConfig = new SparkMaxConfig();

    SmartDashboard.putNumber("kp turret", .08);
    SmartDashboard.putNumber("ki turret", .0);
    SmartDashboard.putNumber("kd turret", .0);

    m_turretSparkMax.configure(
        m_turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turretSparkMax.getEncoder().setPosition(0);
    m_turretController =
        new PIDController(
            SmartDashboard.getNumber("kp turret", .08),
            SmartDashboard.getNumber("ki turret", .0),
            SmartDashboard.getNumber("kd turret", .0)); // TODO: change values
    m_objectTrackerSubsystem = objectTrackerSubsystem;
  }

  public void aimAtTarget(int tag) {
    Pose2d aprilTagVector =
        m_objectTrackerSubsystem.getDistVector(
            0, 0, // -/+   .6m
            0, tag);
    // Pose2d aprilTagVector =
    //     m_objectTrackerSubsystem.getDistVector(
    //         0,
    //         Units.metersToInches(-0.6), // -/+   .6m
    //         0,
    //         tag);
    aprilTagX = aprilTagVector.getX();
    aprilTagY = aprilTagVector.getY();
    aprilTagDelataRot = Math.atan2(aprilTagVector.getY(), aprilTagVector.getX());
    SmartDashboard.putNumber("turretx", aprilTagVector.getX());
    SmartDashboard.putNumber("turrety", aprilTagVector.getY());
    SmartDashboard.putNumber("turretrot", aprilTagVector.getRotation().getDegrees());

    m_objectTrackerSubsystem.data();
    SmartDashboard.putNumber("Camera Z", m_objectTrackerSubsystem.getVisionZ(tag));

    // setTurretTarget(
    //     MathUtil.clamp(
    //         m_poseTarget + Math.atan2(aprilTagVector.getY(),aprilTagVector.getX()),
    //         Constants.MIN_LIMIT_ROTATION,
    //         Constants.MAX_LIMIT_ROTATION));
    SmartDashboard.putNumber(
        "deltaRotTurret", Math.atan2(aprilTagVector.getY(), aprilTagVector.getX()));
  }

  public void turretPower(double power) {
    m_turretSparkMax.setVoltage(power);
  }

  public void resetEncoder() {
    // remember that the setup will be where the turret start
    m_turretSparkMax.getEncoder().setPosition(0);
  }

  public void stopTurret() {
    m_turretSparkMax.setVoltage(0);
  }

  public double getEncoder() {
    return m_turretSparkMax.getEncoder().getPosition();
  }

  public double getDegrees() {
    double deg =
        (getEncoder() / 7.5)
            // *(Constants.RATIO_SPARKMAX_ROTATION_TO_TURRET
            //     / Constants.ENCODER_TICS_PER_SPARKMAX_REVOLUTION))
            * 90;
    return deg % 360;
  }

  public boolean isTurretAtTarget() {
    return Math.abs(getDegrees() - m_poseTarget) < Constants.TURRET_TOLERANCE;
  }

  public void moveTurretLeft() {

    m_poseTarget -= Constants.TURRET_ANGLE_MOVE; // do not know if this is correct (+/-)
  }

  public void moveTurretRight() {
    m_poseTarget += Constants.TURRET_ANGLE_MOVE; // do not know if this is correct (+/-)
  }

  public void setTurretTarget(double position) {
    m_poseTarget =
        MathUtil.clamp(m_poseTarget, Constants.MIN_LIMIT_ROTATION, Constants.MAX_LIMIT_ROTATION);
  }

  public void setAutoControl(boolean state) {
    isAutoControl = state;
  }

  public int getTag() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) {
        return 10;
      } else {
        return 26;
      }
    }
    return 10;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Encoder Counts", getEncoder());
    SmartDashboard.putNumber("Turret Degrees", getDegrees());
    m_poseTarget = MathUtil.clamp(m_poseTarget, -45, 45);
    double fb = m_turretController.calculate(getDegrees(), m_poseTarget);
    SmartDashboard.putNumber("Feed Back", fb);
    SmartDashboard.putNumber("Pose Target", m_poseTarget);
    SmartDashboard.putNumber("April tag x turret", aprilTagX);
    SmartDashboard.putNumber("April tag y turret", aprilTagY);
    SmartDashboard.putNumber("April tag delta rotation turret", aprilTagDelataRot);
    aimAtTarget(getTag());

    // if(isAutoControl){
    //   aimAtTarget(10);
    // }
    // pid =
    //       MathUtil.clamp(
    //           m_turretController.calculate(getDegrees(), m_poseTarget),
    //           -Constants.TURRET_POWER,
    //           Constants.TURRET_POWER);
    turretPower(fb);
    // turretPower(0);
    // This method will be called once per scheduler run
  }
}
