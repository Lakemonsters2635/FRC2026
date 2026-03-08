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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
  public double aprilTagDeltaRot = 0;
  private double feedForward = 0;

  public TurretSubsystem(ObjectTrackerSubsystem objectTrackerSubsystem) {
    m_turretSparkMax = new SparkMax(Constants.TURRET_MOTOR_ID, MotorType.kBrushless);
    m_turretConfig = new SparkMaxConfig();

    SmartDashboard.putNumber("kp turret", .08);
    SmartDashboard.putNumber("ki turret", .0);
    SmartDashboard.putNumber("kd turret", .0);

    m_turretSparkMax.configure(
        m_turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turretSparkMax.getEncoder().setPosition(0);
    m_turretController = new PIDController(0.07, 0.0, 0); // TODO: change values
    m_objectTrackerSubsystem = objectTrackerSubsystem;
  }

  public void aimAtTarget(int tag) {
    Pose2d aprilTagVector =
        m_objectTrackerSubsystem.getDistVector(
            0,
            Units.metersToInches(.6), // -/+   .6m
            0,
            tag);
    // Pose2d aprilTagVector =
    //     m_objectTrackerSubsystem.getDistVector(
    //         0,
    //         Units.metersToInches(-0.6), // -/+   .6m
    //         0,
    //         tag);
    aprilTagX = aprilTagVector.getX();
    aprilTagY = -aprilTagVector.getY();
    if (!aprilTagVector.equals(new Pose2d(0, 0, new Rotation2d(0)))) {
      aprilTagDeltaRot =
          Units.radiansToDegrees(
              Math.atan2(aprilTagVector.getY(), aprilTagVector.getX()) + Math.PI / 2);
    } else {
      aprilTagDeltaRot = 0;
    }
    setTurretTarget(getDegrees() + aprilTagDeltaRot);
    SmartDashboard.putNumber("turretx", aprilTagVector.getX());
    SmartDashboard.putNumber("turrety", aprilTagVector.getY());
    SmartDashboard.putNumber("turretrot", aprilTagVector.getRotation().getDegrees());

    m_objectTrackerSubsystem.data();

    // setTurretTarget(
    //     MathUtil.clamp(
    //         m_poseTarget + Math.atan2(aprilTagVector.getY(),aprilTagVector.getX()),
    //         Constants.MIN_LIMIT_ROTATION,
    //         Constants.MAX_LIMIT_ROTATION));
    SmartDashboard.putNumber("deltaRotTurret", aprilTagDeltaRot);
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
    return deg;
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
        MathUtil.clamp(position, Constants.MIN_LIMIT_ROTATION, Constants.MAX_LIMIT_ROTATION);
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
    m_poseTarget = MathUtil.clamp(m_poseTarget, -60, 60);
    double fb = MathUtil.clamp(m_turretController.calculate(getDegrees(), m_poseTarget), -5, 5);
    SmartDashboard.putNumber("Feed Back", fb);
    SmartDashboard.putNumber("Pose Target", m_poseTarget);
    SmartDashboard.putNumber("April tag x turret", aprilTagX);
    SmartDashboard.putNumber("April tag y turret", aprilTagY);
    SmartDashboard.putNumber("April tag delta rotation turret", aprilTagDeltaRot);
    aimAtTarget(10);
    // aimAtTarget(getTag());

    // if(isAutoControl){
    //   aimAtTarget(10);
    // }
    // pid =
    //       MathUtil.clamp(
    //           m_turretController.calculate(getDegrees(), m_poseTarget),
    //           -Constants.TURRET_POWER,
    //           Constants.TURRET_POWER);
    if (getDegrees() > -30 && getDegrees() < 17) {
      feedForward = 0;
    } else if (getDegrees() > 17) {
      feedForward = 0.7 * (getDegrees() - 17) / (Constants.MAX_LIMIT_ROTATION - 17);
    } else {
      feedForward = -0.7 * (getDegrees() + 30) / (Constants.MIN_LIMIT_ROTATION + 30);
    }

    if (Math.abs(getDegrees()) > 62) {
      turretPower(0);
    } else {
      turretPower(-fb + feedForward);
    }
    SmartDashboard.putNumber("Feed Forward Turret", feedForward);
    // turretPower(feedForward);

    // turretPower(0);
    // This method will be called once per scheduler run
  }
}
