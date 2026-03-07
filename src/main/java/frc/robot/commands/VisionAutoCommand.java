package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Detection;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;

public class VisionAutoCommand extends Command {
  private final DrivetrainSubsystem m_dts;
  private final ObjectTrackerSubsystem m_ots;

  private final int m_tagID;
  private int m_nearestTagID;
  // private final String YOLO_object;

  private final double m_xPrime, m_zPrime, m_finalYa;
  private double m_x_target;
  private double m_y_target;
  private double m_rot_target;

  private double m_cameraRotation;
  Pose2d targetPose;

  boolean finishIt = false;

  double x_pose_last_check;
  double y_pose_last_check;
  double rot_pose_last_check;

  Pose2d tempTargetPose;

  // Some of the variables arent used outside of the constructor so can be deleted but keeping it
  // for debugging and future use
  private double m_x_start, m_y_start, m_rot_start;
  boolean isReachX = false, isReachY = false, isReachRot = false;

  boolean isNearestAuto = false;

  // Probably need to fine tune constants depending on the bot
  private final PIDController m_visionSwerveController_x = new PIDController(10, 0, 0);
  private final PIDController m_visionSwerveController_y = new PIDController(10, 0, 0);
  private final PIDController m_visionSwerveController_rot = new PIDController(4, 0, 0);

  private static final double PURE_VISION_MAX_M_PER_SEC = 2;
  private static final double PURE_VISION_MAX_RAD_PER_SEC = Math.PI;

  private final Trigger cancelTeleAuto =
      new JoystickButton(RobotContainer.rightJoystick, 6); // Button in case visionAuto goes haywire

  // Constructor for a visionAutoCommand
  public VisionAutoCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      ObjectTrackerSubsystem objectTrackerSubsystem,
      int tagID,
      double xPrime,
      double zPrime,
      double finalYa,
      double cameraRotation,
      boolean isNearestAuto) {
    m_dts = drivetrainSubsystem;
    m_ots = objectTrackerSubsystem;
    this.m_tagID = tagID;
    m_xPrime = xPrime;
    m_zPrime = zPrime;
    m_finalYa = finalYa;
    m_cameraRotation = cameraRotation;
    this.isNearestAuto = isNearestAuto;
    // YOLO_object = yoloObject;

    //   targetPose = visionAutoData(xPrime, zPrime, finalYa, tagID);
    //   m_x_target = targetPose.getX();
    //   m_y_target = targetPose.getY();
    //   m_rot_target = targetPose.getRotation().getDegrees();

    addRequirements(m_dts, m_ots);
  }

  @Override
  public void initialize() {
    try {
      m_nearestTagID =
          Integer.parseInt(m_ots.getNearestAprilTagDetection().objectLabel.substring(10));
    } catch (Exception e) {
      finishIt = true;
    }
    isReachRot = false;
    isReachX = false;
    isReachY = false;
    m_dts.stashAngle();
    // Order in these resets has caused problems in the past. This should be correct.
    m_dts.resetAngle();
    m_dts.zeroOdometry();

    m_x_start = m_dts.getPose().getX();
    m_y_start = m_dts.getPose().getY();
    m_rot_start = m_dts.getPose().getRotation().getDegrees();

    m_dts.setFollowJoystick(false);
    m_dts.stopMotors();

    updateDashboard();

    x_pose_last_check = m_dts.getPose().getX();
    y_pose_last_check = m_dts.getPose().getY();
    rot_pose_last_check = m_dts.getPose().getRotation().getDegrees();

    m_visionSwerveController_rot.enableContinuousInput(-180, 180);
  }

  @Override
  public void execute() {
    try {
      m_ots.data();
      if (isNearestAuto) {
        tempTargetPose = visionAutoData(m_xPrime, m_zPrime, m_finalYa, m_nearestTagID);
      } else {
        tempTargetPose = visionAutoData(m_xPrime, m_zPrime, m_finalYa, m_tagID);
      }
      SmartDashboard.putBoolean("isNull", tempTargetPose == null);
      // This if statement will run only if we are able to see the april tag
      if (tempTargetPose != null) {
        targetPose = tempTargetPose;

        m_x_target = targetPose.getX();
        m_y_target = targetPose.getY();
        m_rot_target = targetPose.getRotation().getDegrees();

        // Stores the latest robot position we were able to see the april tag
        x_pose_last_check = m_dts.getPose().getX();
        y_pose_last_check = m_dts.getPose().getY();
        rot_pose_last_check = m_dts.getPose().getRotation().getDegrees();
      } else {
        targetPose = new Pose2d(0, 0, new Rotation2d(0));
      }
    } catch (Exception e) {
      System.out.println(e);
    }

    double x_pose = m_dts.getPose().getX();
    double y_pose = m_dts.getPose().getY();
    double rot_pose = m_dts.getPose().getRotation().getDegrees();

    /*
     * x_pose= robot's current x position
     * When see april tag:
     * x_pose = x_pose_last_check
     *
     * When not see april tag:
     * x_pose_last_check = last robot position when able to see apriltag
     */

    SmartDashboard.putNumber("deltaPosePosex", x_pose - x_pose_last_check);
    SmartDashboard.putNumber("deltaPosePosey", y_pose - y_pose_last_check);

    SmartDashboard.putNumber("m_y_target", m_y_target);
    SmartDashboard.putNumber("x_last_check", x_pose_last_check);
    // TODO: Use cameraAngleOffset and a rotation matrix for conversions
    // Note the negatives in the equations below change this to a -90deg rotation
    // .calculate(measurement, setpoint)
    Translation2d transformedTargetPose = applyRotationMatrix(m_x_target, m_y_target);
    double pid_x =
        m_visionSwerveController_x.calculate(
            x_pose, transformedTargetPose.getX() + x_pose_last_check);
    double pid_y =
        m_visionSwerveController_y.calculate(
            y_pose, transformedTargetPose.getY() + y_pose_last_check);
    double pid_rot =
        m_visionSwerveController_rot.calculate(
            Math.toRadians(rot_pose), Math.toRadians((m_rot_target + rot_pose_last_check) % 360));
    if (Math.abs(m_visionSwerveController_x.getError()) < 0.02) isReachX = true;

    if (Math.abs(m_visionSwerveController_y.getError()) < 0.02) isReachY = true;
    if (Math.abs(Math.toDegrees(m_visionSwerveController_rot.getError())) < 3) {
      isReachRot = true;
    }
    SmartDashboard.putNumber("rot_error", Math.toDegrees(m_visionSwerveController_rot.getError()));
    SmartDashboard.putNumber("m_rot_target", m_rot_target);
    SmartDashboard.putNumber("rot_pose_last_check", rot_pose_last_check);
    SmartDashboard.putNumber("transformedTargetPoseY", transformedTargetPose.getY());
    SmartDashboard.putNumber("transformedTargetPoseX", transformedTargetPose.getX());

    SmartDashboard.putNumber(
        "deltaposetargetx",
        m_visionSwerveController_x.getError()); // -x_pose-(m_y_target - x_pose_last_check)
    SmartDashboard.putNumber("deltaposetargety", m_visionSwerveController_y.getError());
    SmartDashboard.putNumber("pidxtarget", m_y_target - x_pose_last_check);
    SmartDashboard.putNumber("pidytarget", -m_x_target + y_pose_last_check);

    SmartDashboard.putBoolean("isReachRot", isReachRot);
    SmartDashboard.putBoolean("isReachX", isReachX);
    SmartDashboard.putBoolean("isReachY", isReachY);

    pid_rot = isReachRot ? 0 : pid_rot;
    if (isReachRot && isReachX) {
      pid_x = 0;
    }
    if (isReachRot && isReachY) {
      pid_y = 0;
    }
    double pid_c = Math.hypot(pid_x, pid_y);

    SmartDashboard.putNumber("deltaposetargetRot", m_visionSwerveController_rot.getError());
    SmartDashboard.putNumber("deltaposetargetx", m_visionSwerveController_x.getError());
    // double pid_c_clamp =
    // double x_clamp = pid_c > speed_clamp ? (speed_clamp * (Math.abs(pid_x) / pid_c)) :
    // speed_clamp;
    // double y_clamp = pid_c > speed_clamp ? (speed_clamp * (Math.abs(pid_y) / pid_c)) :
    // speed_clamp;

    // Clamps for safety and allows the fadeInDistance to ramp up speed instead of instantly
    // accelerating too fast
    // double m_fb_x = MathUtil.clamp(pid_x, -x_clamp, x_clamp);
    // double m_fb_y = MathUtil.clamp(pid_y, -y_clamp, y_clamp);
    // double m_fb_rot = MathUtil.clamp(pid_rot, -PURE_VISION_MAX_RAD_PER_SEC,
    // PURE_VISION_MAX_RAD_PER_SEC);

    double m_fb_x = MathUtil.clamp(pid_x / pid_c * 1.5, -1.5, 1.5);
    double m_fb_y = MathUtil.clamp(pid_y / pid_c * 1.5, -1.5, 1.5);
    double m_fb_rot = MathUtil.clamp(pid_rot / 2, -Math.PI / 2, Math.PI / 2);

    SmartDashboard.putNumber("pid_x", pid_x);
    SmartDashboard.putNumber("pid_y", pid_y);

    SmartDashboard.putNumber("m_fb_x", m_fb_x);
    SmartDashboard.putNumber("m_fb_y", m_fb_y);
    m_dts.drive(m_fb_x, m_fb_y, m_fb_rot, true);
    updateDashboard();
    // m_dts.drive(m_fb_x, m_fb_y, m_fb_rot, true);
  }

  @Override
  public void end(boolean interrupted) {
    m_dts.restoreAngle();
    m_dts.stopMotors();
    m_dts.setFollowJoystick(true);
    // m_dts.setStopVisionAutoCommand(false);
  }

  @Override
  public boolean isFinished() {
    double x = m_dts.getPose().getX();
    double y = m_dts.getPose().getY();
    double rot = m_dts.getPose().getRotation().getDegrees();

    // Manual Stop by Driver
    if (cancelTeleAuto.getAsBoolean() || finishIt) {
      return true;
    }

    // If conditions are met
    return isReachRot && isReachX && isReachY;
    // return Math.abs(m_visionSwerveController_x.getError()) < 0.04 &&
    //        Math.abs(m_visionSwerveController_y.getError()) < 0.04 &&
    // // Math.abs(m_y_target-(x_pose_last_check-x)) < 0.04 &&
    //       //  Math.abs(y-(-m_x_target + y_pose_last_check)) < 0.04 &&
    //        Math.abs(m_visionSwerveController_rot.getError()) < (3.0 / 180.0 * Math.PI);
    //       //  && Math.abs(((m_rot_target + rot_pose_last_check) % 360 - rot) % 360) < 3;
    //       //  && Math.abs(m_dts.getYawGyroValue()) < 10;
  }

  private Pose2d visionAutoData(double xPrime, double zPrime, double finalYa, int tagId) {
    Detection detection = null;
    for (int i = 0; i < 100; i++) { // TODO: do we still want to keep for loop?
      try {
        detection = m_ots.getSpecificAprilTag(tagId);
        break;
      } catch (Exception e) {
        System.out.println("Failed vision attempt " + i);
      }
    }
    if (detection == null) {
      SmartDashboard.putBoolean("ableToSeeAT", false);
      return null;
    }

    SmartDashboard.putBoolean("ableToSeeAT", true);

    // detection = m_ots.

    double visionYa = -detection.ya;
    double x_vt =
        xPrime * Math.cos(Math.toRadians(visionYa)) - zPrime * Math.sin(Math.toRadians(visionYa));
    double z_vt =
        xPrime * Math.sin(Math.toRadians(visionYa)) + zPrime * Math.cos(Math.toRadians(visionYa));
    // Should be offset variables and change based of camera location relative to the center of the
    // robot
    double deltaRobotX = -(detection.x + x_vt - Constants.CAM_X_OFFSET);
    double deltaRobotY = -(detection.z + z_vt - Constants.CAM_Y_OFFSET);

    // double deltaRobotX = -(detection.x + x_vt - Constants.VISION_TOTE_CAM_OFFSET[0]);
    // double deltaRobotY = -(detection.z + z_vt - Constants.VISION_TOTE_CAM_OFFSET[1]);

    double botRadians = 0; // Units.degreesToRadians(m_dts.getPose().getRotation().getDegrees());
    double deltaFieldX = deltaRobotX * Math.cos(botRadians) - deltaRobotY * Math.sin(botRadians);
    double deltaFieldY = deltaRobotX * Math.sin(botRadians) + deltaRobotY * Math.cos(botRadians);
    double finalAngle = visionYa + finalYa + Units.radiansToDegrees(botRadians);

    return new Pose2d(
        Units.inchesToMeters(deltaFieldX),
        Units.inchesToMeters(deltaFieldY),
        new Rotation2d(Units.degreesToRadians(finalAngle)));
  }

  private Pose2d nearestVisionAutoData(double xPrime, double zPrime, double finalYa) {
    Detection detection = null;
    for (int i = 0; i < 100; i++) { // TODO: do we still want to keep for loop?
      try {
        detection = m_ots.getNearestAprilTagDetection();
        break;
      } catch (Exception e) {
        System.out.println("Failed vision attempt " + i);
      }
    }
    if (detection == null) {
      SmartDashboard.putBoolean("ableToSeeAT", false);
      return null;
    }

    SmartDashboard.putBoolean("ableToSeeAT", true);

    // detection = m_ots.

    double visionYa = -detection.ya;
    double x_vt =
        xPrime * Math.cos(Math.toRadians(visionYa)) - zPrime * Math.sin(Math.toRadians(visionYa));
    double z_vt =
        xPrime * Math.sin(Math.toRadians(visionYa)) + zPrime * Math.cos(Math.toRadians(visionYa));
    // Should be offset variables and change based of camera location relative to the center of the
    // robot
    double deltaRobotX = -(detection.x + x_vt - Constants.CAM_X_OFFSET);
    double deltaRobotY = -(detection.z + z_vt - Constants.CAM_Y_OFFSET);

    // double deltaRobotX = -(detection.x + x_vt - Constants.VISION_TOTE_CAM_OFFSET[0]);
    // double deltaRobotY = -(detection.z + z_vt - Constants.VISION_TOTE_CAM_OFFSET[1]);

    double botRadians = 0; // Units.degreesToRadians(m_dts.getPose().getRotation().getDegrees());
    double deltaFieldX = deltaRobotX * Math.cos(botRadians) - deltaRobotY * Math.sin(botRadians);
    double deltaFieldY = deltaRobotX * Math.sin(botRadians) + deltaRobotY * Math.cos(botRadians);
    double finalAngle = visionYa + finalYa + Units.radiansToDegrees(botRadians);

    return new Pose2d(
        Units.inchesToMeters(deltaFieldX),
        Units.inchesToMeters(deltaFieldY),
        new Rotation2d(Units.degreesToRadians(finalAngle)));
  }

  public Translation2d applyRotationMatrix(double targetX, double targetY) {
    // targetY is y for rotation matrix
    // targetX is x for rotation  matrix

    // transformedX = xcos(theta) - ysin(theta)
    // transfromedY = xsin(theta) + ycos(theta)

    double transformedX =
        (targetX * Math.cos(Math.toRadians(m_cameraRotation)))
            - (targetY * Math.sin(Math.toRadians(m_cameraRotation)));
    double transformedY =
        (targetX * Math.sin(Math.toRadians(m_cameraRotation)))
            + (targetY * Math.cos(Math.toRadians(m_cameraRotation)));

    // the rotation does not matter because we use finalYa
    // the transfromedX and Y are used instead of
    return new Translation2d(transformedX, transformedY);
  }

  private void updateDashboard() {
    SmartDashboard.putNumber("xPidTargetVision", m_x_target);
    SmartDashboard.putNumber("yPidTargetVision", m_y_target);
    SmartDashboard.putNumber("rotPidTargetVision", m_rot_target);
    SmartDashboard.putNumber("xPrime", m_xPrime);
    SmartDashboard.putNumber("zPrime", m_zPrime);
    SmartDashboard.putNumber("finalYa", m_finalYa);
    SmartDashboard.putNumber("m_tagId", m_tagID);
    SmartDashboard.putNumber("xPidStart", m_x_start);
    SmartDashboard.putNumber("yPidStart", m_y_start);
    SmartDashboard.putNumber("rotPidStart", m_rot_start);
    // SmartDashboard.putBoolean("isVisionAuto", isVisionAuto);
  }
}
