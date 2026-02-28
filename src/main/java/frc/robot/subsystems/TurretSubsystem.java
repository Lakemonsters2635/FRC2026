// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.VisionAutoCommand;

public class TurretSubsystem extends SubsystemBase {
  /** Creates a new TurretSubsystem. */
  ObjectTrackerSubsystem m_objectTrackerSubsystem;

  VisionAutoCommand m_visionAutoCommand;
  private final SparkMax m_turretSparkMax;
  PIDController m_turretController;
  double pid;
  public double m_poseTarget;

  public TurretSubsystem(ObjectTrackerSubsystem objectTrackerSubsystem) {
    m_turretSparkMax = new SparkMax(Constants.TURRET_MOTOR_ID, MotorType.kBrushless);
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
    double angleToAPOffset = 0;
    double finalAngle = 0;
    Pose2d aprilTagVector =
        m_objectTrackerSubsystem.visionAutoData(
            m_objectTrackerSubsystem.getVisionX(tag),
            m_objectTrackerSubsystem.getVisionZ(tag),
            m_objectTrackerSubsystem.getVisionYa(tag),
            tag);
    double aprilTagVectorX = aprilTagVector.getX();
    double aprilTagVectorY = aprilTagVector.getY();
    double aprilTagVectorAngle = Math.toDegrees(Math.atan(aprilTagVectorY / aprilTagVectorX));
    double aprilTagOffsetVectorAngle =
        Math.toDegrees(
            Math.atan((aprilTagVectorY + Constants.APRIL_TAG_AIM_OFFSET) / aprilTagVectorX));
    angleToAPOffset = aprilTagOffsetVectorAngle - aprilTagVectorAngle;
    // We set the pose equal to the angle to the april tag combined with the angle to the ideal
    // vector from the april tag plus the current angle
    finalAngle = (angleToAPOffset + angleToAP) + getDegrees();
    if (finalAngle >= Constants.MIN_LIMIT_ROTATION && finalAngle <= Constants.MAX_LIMIT_ROTATION) {
      setTurretTarget(finalAngle);
    }
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
        (getEncoder()
                / (Constants.RATIO_SPARKMAX_ROTATION_TO_TURRET
                    * Constants.ENCODER_TICS_PER_SPARKMAX_REVOLUTION))
            * 360;
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
    if (position > Constants.MIN_LIMIT_ROTATION && position < Constants.MAX_LIMIT_ROTATION) {
      m_poseTarget = position;
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Encoder Counts", getEncoder());
    SmartDashboard.putNumber("Turret Degrees", getDegrees());
    pid =
        MathUtil.clamp(
            m_turretController.calculate(getDegrees(), m_poseTarget),
            -Constants.TURRET_POWER,
            Constants.TURRET_POWER);
    turretPower(pid);
    // This method will be called once per scheduler run
  }
}
