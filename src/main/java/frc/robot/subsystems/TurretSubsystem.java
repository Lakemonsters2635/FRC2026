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
import edu.wpi.first.wpilibj.Timer;
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
  public double aprilTagDeltaRot = 0; // target
  public double m_poseTarget_prev1 = 0;
  public double m_poseTarget_prev2 = 0;
  public double m_poseTarget_prev3 = 0;
  public double m_poseTarget_prev4 = 0;
  private double feedForward = 0;
  private double savePose = 0;
  Timer m_timer;
  private double waitTime = 25; // each increment by 1 is an additional 20 mileseconds
  private double time = 0;
  boolean isMidMode = false;

  public TurretSubsystem(ObjectTrackerSubsystem objectTrackerSubsystem) {
    m_turretSparkMax = new SparkMax(Constants.TURRET_MOTOR_ID, MotorType.kBrushless);
    m_turretConfig = new SparkMaxConfig();

    m_turretConfig.smartCurrentLimit(20);

    m_turretSparkMax.configure(
        m_turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turretSparkMax.getEncoder().setPosition(0);
    m_turretController =
        new PIDController(
            0.10, 0.,
            0.01); // Having a kd of 0.01 does induce some jitter but greatly reduces overshoot
    m_timer = new Timer();
    m_objectTrackerSubsystem = objectTrackerSubsystem;
  }

  public boolean checkIsAutoControlValid() {
    for (int i = 0; i < Constants.APRIL_TAGS_HUB.length; i++) {
      if (Constants.APRIL_TAGS_HUB[i] == m_objectTrackerSubsystem.getNearestAprilTag()) {
        return true;
      }
    }
    return false;
  }

  public void aimAtTarget() {
    m_objectTrackerSubsystem.data();

    Pose2d aprilTagVector = m_objectTrackerSubsystem.getNearestAprilTagDistTurret();
    //     m_objectTrackerSubsystem.getDistVector(
    //         0,
    //         Units.metersToInches(0.8), // -/+   .6m
    //         0,
    //         tag);
    // // Pose2d aprilTagVector =
    //     m_objectTrackerSubsystem.getDistVector(
    //         0,
    //         Units.metersToInches(-0.6), // -/+   .6m
    //         0,
    //         tag);
    aprilTagX = -aprilTagVector.getX();
    aprilTagY = -aprilTagVector.getY();
    double pose_target = 0;
    if (!aprilTagVector.equals(new Pose2d(0, 0, new Rotation2d(0)))) {
      aprilTagDeltaRot = Units.radiansToDegrees(Math.atan2(aprilTagY, aprilTagX) - Math.PI / 2);
      pose_target = getDegrees() - aprilTagDeltaRot;
      // m_poseTarget_prev1 = pose_target;
      // m_poseTarget_prev2 = pose_target;
      // m_poseTarget_prev3 = pose_target;
      // m_poseTarget_prev4 = pose_target;
      // setTurretTarget(pose_target);
      time = 0;
      savePose = pose_target;
      SmartDashboard.putNumber("tur: pose_target aimAtTarget", pose_target);

    } else {
      time++;
      if (time < waitTime) {
        pose_target = savePose;
      } else {
        pose_target = getDegrees();
      }

      // pose_target = m_poseTarget_prev1;
      // m_poseTarget_prev1 = m_poseTarget_prev2;
      // m_poseTarget_prev2 = m_poseTarget_prev3;
      // m_poseTarget_prev3 = m_poseTarget_prev4;
      // m_poseTarget_prev4 = 0; // goes back to straight if we are unable to see for about 80ms
    }

    // aprilTagDeltaRot is positive ccw, getDegrees is positive cw
    // Substracting off the deltaRot gives us the correct target turret pos in the turret frame of
    // reference
    // TODO: Refactor everything to make coordinates aligned

    setTurretTarget(pose_target);
    SmartDashboard.putNumber("tur: vision targetx", aprilTagX);
    SmartDashboard.putNumber("tur: vision targety", aprilTagY);
    SmartDashboard.putNumber(
        "tur: vision dist to target", Math.sqrt((aprilTagX * aprilTagX + aprilTagY * aprilTagY)));
    SmartDashboard.putNumber("tur: deltaRotTurret", aprilTagDeltaRot);
    SmartDashboard.putNumber("tur: getDegrees", getDegrees());
    SmartDashboard.putNumber("tur: current", m_turretSparkMax.getOutputCurrent());

    // setTurretTarget(
    //     MathUtil.clamp(
    //         m_poseTarget + Math.atan2(aprilTagVector.getY(),aprilTagVector.getX()),
    //         Constants.MIN_LIMIT_ROTATION,
    //         Constants.MAX_LIMIT_ROTATION));
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

  public void setMidMode(boolean val) {
    isMidMode = val;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Encoder Counts", getEncoder());
    SmartDashboard.putNumber("Turret Degrees", getDegrees());
    m_poseTarget = MathUtil.clamp(m_poseTarget, -60, 60);
    double fb = MathUtil.clamp(m_turretController.calculate(getDegrees(), m_poseTarget), -5, 5);
    SmartDashboard.putNumber("Fee d Back", fb);
    SmartDashboard.putNumber("Pose Target", m_poseTarget);
    SmartDashboard.putNumber("April tag x turret", aprilTagX);
    SmartDashboard.putNumber("April tag y turret", aprilTagY);
    SmartDashboard.putNumber("April tag delta rotation turret", aprilTagDeltaRot);
    SmartDashboard.putNumber(
        "tur: vision dist to target", Math.sqrt((aprilTagX * aprilTagX + aprilTagY * aprilTagY)));
    if (!isMidMode) {
      if (!checkIsAutoControlValid()) {
        if (!m_timer.isRunning()) {
          m_timer.reset();
          m_timer.start();
        }
        if (m_timer.get() > 1) {
          isAutoControl = false;
        }
      }
      if (checkIsAutoControlValid()) {
        m_timer.stop();

        isAutoControl = true;
      }
      if (isAutoControl) {
        aimAtTarget();
      } else {
        m_poseTarget = 0;
      }
    } else {
      m_poseTarget = 0;
    }

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

    if (Math.abs(getDegrees()) > 50) {
      turretPower(0);
    } else {
      turretPower(fb + feedForward);
      // turretPower(0);
    }
    SmartDashboard.putNumber("Feed Forward Turret", feedForward);
    SmartDashboard.putBoolean("tur: isMidMode", isMidMode);
    SmartDashboard.putBoolean("tur: isAutoControl", isAutoControl);
    // turretPower(feedForward);

    // This method will be called once per scheduler run
  }
}
