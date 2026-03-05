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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.VisionAutoCommand;

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

  public TurretSubsystem(ObjectTrackerSubsystem objectTrackerSubsystem) {
    m_turretSparkMax = new SparkMax(Constants.TURRET_MOTOR_ID, MotorType.kBrushless);
    m_turretConfig = new SparkMaxConfig();

    m_turretSparkMax.configure(
        m_turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turretSparkMax.getEncoder().setPosition(0);
    m_turretController = new PIDController(0.08, 0, 0); // TODO: change values
    m_objectTrackerSubsystem = objectTrackerSubsystem;
  }

  public void aimAtTarget(int tag) {
    // This is the angle that needs to be added to the turrets current angle to make the turret aim
    // at the april tag
    double angleToAP = m_objectTrackerSubsystem.getVisionYa(tag);
    // This is the angle between the april tags one to the bot and the one for the ideal offset
    // vector

    // id9 .356m
    double angleToAPOffset = 0;
    double finalAngle = 0;
    Pose2d aprilTagVector =
        m_objectTrackerSubsystem.getDistVector(
            0,
            Units.metersToInches(.6), // -/+   .6m
            0,
            tag);
    SmartDashboard.putNumber("turretx", aprilTagVector.getX());
    SmartDashboard.putNumber("turrety", aprilTagVector.getY());
    SmartDashboard.putNumber("turretrot", aprilTagVector.getRotation().getDegrees());
    // We shouldn't need any of these code:
    // double aprilTagVectorX = aprilTagVector.getX();
    // double aprilTagVectorY = aprilTagVector.getY();
    // double aprilTagVectorAngle = Math.toDegrees(Math.atan(aprilTagVectorY / aprilTagVectorX));
    // double aprilTagOffsetVectorAngle =
    //     Math.toDegrees(
    //         Math.atan(
    //             (aprilTagVectorY + Constants.APRIL_TAG_AIM_OFFSET)
    //                 / aprilTagVectorX)); // TODO: check if + or - APRIL_TAG_AIM_OFFSET
    // angleToAPOffset = aprilTagOffsetVectorAngle - aprilTagVectorAngle;
    // // We set the pose equal to the angle to the april tag combined with the angle to the ideal
    // // vector from the april tag plus the current angle
    // finalAngle = (angleToAPOffset + angleToAP) + getDegrees();

    setTurretTarget(
        MathUtil.clamp(
            m_poseTarget + Math.atan(aprilTagVector.getY() / aprilTagVector.getX()),
            Constants.MIN_LIMIT_ROTATION,
            Constants.MAX_LIMIT_ROTATION));
    SmartDashboard.putNumber(
        "deltaRotTurret", Math.atan(aprilTagVector.getY() / aprilTagVector.getX()));
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
        (getEncoder() / 7.2)
            // *(Constants.RATIO_SPARKMAX_ROTATION_TO_TURRET
            //     / Constants.ENCODER_TICS_PER_SPARKMAX_REVOLUTION))
            * 90;
    return deg % 360;
  }

  public void moveTurretLeft() {
    setTurretTarget(
        m_poseTarget - Constants.TURRET_ANGLE_MOVE); // do not know if this is correct (+/-)
  }

  public void moveTurretRight() {
    setTurretTarget(
        m_poseTarget + Constants.TURRET_ANGLE_MOVE); // do not know if this is correct (+/-)
  }

  public void setTurretTarget(double position) {
    m_poseTarget =
        MathUtil.clamp(m_poseTarget, Constants.MIN_LIMIT_ROTATION, Constants.MAX_LIMIT_ROTATION);
  }

  public void setAutoControl(boolean state) {
    isAutoControl = state;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Encoder Counts", getEncoder());
    SmartDashboard.putNumber("Turret Degrees", getDegrees());
    aimAtTarget(11);

    // if(isAutoControl){
    //   aimAtTarget(10);
    // }
    // pid =
    //       MathUtil.clamp(
    //           m_turretController.calculate(getDegrees(), m_poseTarget),
    //           -Constants.TURRET_POWER,
    //           Constants.TURRET_POWER);
    //   turretPower(pid);
    turretPower(0);
    // This method will be called once per scheduler run
  }
}
