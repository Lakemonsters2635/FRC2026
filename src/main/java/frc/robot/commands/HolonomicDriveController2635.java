// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This holonomic drive controller can be used to follow trajectories using a holonomic drivetrain
 * (i.e. swerve or mecanum). Holonomic trajectory following is a much simpler problem to solve
 * compared to skid-steer style drivetrains because it is possible to individually control forward,
 * sideways, and angular velocity.
 *
 * <p>The holonomic drive controller takes in one PID controller for each direction, forward and
 * sideways, and one profiled PID controller for the angular direction. Because the heading dynamics
 * are decoupled from translations, users can specify a custom heading that the drivetrain should
 * point toward. This heading reference is profiled for smoothness.
 */
public class HolonomicDriveController2635 extends HolonomicDriveController{
  /**
   * Constructs a holonomic drive controller.
   *
   * @param xController A PID Controller to respond to error in the field-relative x direction.
   * @param yController A PID Controller to respond to error in the field-relative y direction.
   * @param thetaController A profiled PID controller to respond to error in angle.
   */
  public HolonomicDriveController2635(
      PIDController xController, PIDController yController, ProfiledPIDController thetaController) {
    super(xController, yController, thetaController);
    // m_xController = xController;
    // m_yController = yController;
    // m_thetaController = thetaController;
    // m_thetaController.enableContinuousInput(0, Units.degreesToRadians(360.0));
  }

  @Override
  /**
   * Returns the next output of the holonomic drive controller.
   *
   * @param currentPose The current pose, as measured by odometry or pose estimator.
   * @param trajectoryPose The desired trajectory pose, as sampled for the current timestep.
   * @param desiredLinearVelocityMetersPerSecond The desired linear velocity.
   * @param desiredHeading The desired heading.
   * @return The next output of the holonomic drive controller.
   */
  public ChassisSpeeds calculate(
      Pose2d currentPose,
      Pose2d trajectoryPose,
      double desiredLinearVelocityMetersPerSecond,
      Rotation2d desiredHeading) {
    ChassisSpeeds returnVal = super.calculate(currentPose, trajectoryPose, desiredLinearVelocityMetersPerSecond, desiredHeading);
    // If this is the first run, then we need to reset the theta controller to the current pose's
    // heading.

    ProfiledPIDController thetaController = super.getThetaController();
    PIDController xController = super.getXController();
    PIDController yController = super.getYController();

    // if (m_firstRun) {
    //     thetaController.reset(currentPose.getRotation().getRadians());
    //     m_firstRun = false;
    // }

    // Calculate feedforward velocities (field-relative).
    double xFF = desiredLinearVelocityMetersPerSecond * trajectoryPose.getRotation().getCos() * 0.95; // Fudge factor of 3/4 to adjust the speed
    double yFF = desiredLinearVelocityMetersPerSecond * trajectoryPose.getRotation().getSin() * 0.95;
    double thetaFF =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), desiredHeading.getRadians());

    // Pose2d poseMeters = new Pose2d(currentPose.getTranslation().div(39.37), currentPose.getRotation());
    Pose2d poseError = trajectoryPose.relativeTo(currentPose);
    Rotation2d rotationError = desiredHeading.minus(currentPose.getRotation());

    // This is only used when we disable the feedback controller
    // if (!m_enabled) {
    //     return ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, thetaFF, currentPose.getRotation());
    // }

    // Calculate feedback velocities (based on position error).
    double xFeedback = xController.calculate(currentPose.getX(), trajectoryPose.getX());
    double yFeedback = yController.calculate(currentPose.getY(), trajectoryPose.getY());
        
    // Return next output.
    SmartDashboard.putNumber("holo xFF", xFF);
    SmartDashboard.putNumber("holo yFF", yFF);
    SmartDashboard.putNumber("holo thetaFF", thetaFF);

    SmartDashboard.putNumber("holo xFeedback", xFeedback);
    SmartDashboard.putNumber("holo yFeedback", yFeedback);

    SmartDashboard.putNumber("holo poseErrorX", poseError.getX());
    SmartDashboard.putNumber("holo poseErrorY", poseError.getY());
    SmartDashboard.putNumber("holo rotationError deg", rotationError.getDegrees());

    SmartDashboard.putNumber("holo traj getX", trajectoryPose.getX());
    SmartDashboard.putNumber("holo traj getY", trajectoryPose.getY());
    SmartDashboard.putNumber("holo traj getRotation", trajectoryPose.getRotation().getRadians());

    SmartDashboard.putNumber("holo curr getX", currentPose.getX());
    SmartDashboard.putNumber("holo curr getY", currentPose.getY());
    SmartDashboard.putNumber("holo curr getRotation", currentPose.getRotation().getRadians());

    // Return next output.
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xFF + xFeedback, yFF + yFeedback, thetaFF, currentPose.getRotation());

    // return returnVal;
    
    //ChassisSpeeds.fromFieldRelativeSpeeds(xFF + xFeedback, yFF + yFeedback, thetaFF, currentPose.getRotation());
  }

  @Override
  /**
   * Returns the next output of the holonomic drive controller.
   *
   * @param currentPose The current pose, as measured by odometry or pose estimator.
   * @param desiredState The desired trajectory pose, as sampled for the current timestep.
   * @param desiredHeading The desired heading.
   * @return The next output of the holonomic drive controller.
   */
  public ChassisSpeeds calculate(
      Pose2d currentPose, Trajectory.State desiredState, Rotation2d desiredHeading) {
    return calculate(
        currentPose, desiredState.poseMeters, desiredState.velocityMetersPerSecond, desiredHeading);
  }
}
