package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Detection;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;

public class PidAutoCommand extends Command {
  private final DrivetrainSubsystem m_dts;
  private final ObjectTrackerSubsystem m_ots;

  private final int m_tagID;
  // private final String YOLO_object;

  private final double m_xPrime, m_zPrime, m_finalYa;
  private double m_x_target;
  private double m_y_target;
  private double m_rot_target;

  // Some of the variables arent used outside of the constructor so can be deleted but keeping it
  // for debugging and future use
  private double m_x_start, m_y_start, m_rot_start;
  private boolean isVisionAuto;

  // Probably need to fine tune constants depending on the bot
  private final PIDController m_visionSwerveController_x = new PIDController(10, 0, 0);
  private final PIDController m_visionSwerveController_y = new PIDController(10, 0, 0);
  private final PIDController m_visionSwerveController_rot = new PIDController(4, 0, 0);

  private static final double PURE_VISION_MAX_M_PER_SEC = 2;
  private static final double PURE_VISION_MAX_RAD_PER_SEC = Math.PI;

  private final Trigger cancelTeleAuto =
      new JoystickButton(RobotContainer.rightJoystick, 6); // Button in case visionAuto goes haywire

  // Constructor for a visionAutoCommand
  public PidAutoCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      ObjectTrackerSubsystem objectTrackerSubsystem,
      int tagID,
      double xPrime,
      double zPrime,
      double finalYa,
      boolean isVisionAuto) {
    m_dts = drivetrainSubsystem;
    m_ots = objectTrackerSubsystem;
    this.m_tagID = tagID;
    m_xPrime = xPrime;
    m_zPrime = zPrime;
    m_finalYa = finalYa;
    this.isVisionAuto = isVisionAuto;
    // YOLO_object = yoloObject;

    Pose2d targetPose =
        isVisionAuto ? visionAutoData(xPrime, zPrime, finalYa, tagID) : m_dts.getPose();
    m_x_target = targetPose.getX();
    m_y_target = targetPose.getY();
    m_rot_target = targetPose.getRotation().getDegrees();

    addRequirements(m_dts, m_ots);
  }

  // Constructor for a manual path (NO VISION)
  public PidAutoCommand(
      DrivetrainSubsystem dts,
      ObjectTrackerSubsystem ots,
      double xTarget,
      double yTarget,
      double rotTarget) {
    this(
        dts, ots, -1, 0, 0, 0,
        false); // Sets visionOffsets to 0 to not intefere with targetPose calcs
    m_x_target = xTarget;
    m_y_target = yTarget;
    m_rot_target = rotTarget;
  }

  @Override
  public void initialize() {
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
  }

  @Override
  public void execute() {
    double x_pose = m_dts.getPose().getX();
    double y_pose = m_dts.getPose().getY();
    double rot_pose = m_dts.getPose().getRotation().getDegrees();

    double distanceFromStart = Math.hypot(x_pose - m_x_start, y_pose - m_y_start);
    // fadeInDistance can be tuned
    double fadeInDistance = 0.5;
    double speed_clamp =
        MathUtil.clamp(
            PURE_VISION_MAX_M_PER_SEC * (distanceFromStart / fadeInDistance),
            0.5,
            PURE_VISION_MAX_M_PER_SEC);

    m_visionSwerveController_rot.enableContinuousInput(-180, 180);

    double pid_x = m_visionSwerveController_x.calculate(x_pose, m_x_target);
    double pid_y = m_visionSwerveController_y.calculate(y_pose, m_y_target);
    double pid_rot =
        m_visionSwerveController_rot.calculate(
            Math.toRadians(rot_pose), Math.toRadians(m_rot_target));

    double pid_c = Math.hypot(pid_x, pid_y);
    double x_clamp = pid_c > speed_clamp ? (speed_clamp * (Math.abs(pid_x) / pid_c)) : speed_clamp;
    double y_clamp = pid_c > speed_clamp ? (speed_clamp * (Math.abs(pid_y) / pid_c)) : speed_clamp;

    // Clamps for safety and allows the fadeInDistance to ramp up speed instead of instantly
    // accelerating too fast
    double m_fb_x = MathUtil.clamp(pid_x, -x_clamp, x_clamp);
    double m_fb_y = MathUtil.clamp(pid_y, -y_clamp, y_clamp);
    double m_fb_rot =
        MathUtil.clamp(pid_rot, -PURE_VISION_MAX_RAD_PER_SEC, PURE_VISION_MAX_RAD_PER_SEC);

    m_dts.drive(m_fb_x, m_fb_y, m_fb_rot, true);
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
    if (cancelTeleAuto.getAsBoolean()) {
      return true;
    }

    // If conditions are met
    return Math.abs(m_x_target - x) < 0.05
        && Math.abs(m_y_target - y) < 0.05
        && Math.abs((m_rot_target - rot) % 360) < 5;
    //  && Math.abs(m_dts.getYawGyroValue()) < 10;
  }

  private Pose2d visionAutoData(double xPrime, double zPrime, double finalYa, int tagId) {
    Detection detection = null;
    for (int i = 0; i < 100; i++) {
      try {
        // detection = YOLO_object == null ? (tagId == -1 ? m_ots.getNearestAprilTagDetection() :
        // m_ots.getSpecificAprilTag(tagId)) : m_ots.getNearestYoloDetection();
        break;
      } catch (Exception e) {
        System.out.println("Failed vision attempt " + i);
      }
    }
    if (detection == null) {
      return new Pose2d();
    }

    double visionYa = -detection.ya;
    double x_vt =
        xPrime * Math.cos(Math.toRadians(visionYa)) - zPrime * Math.sin(Math.toRadians(visionYa));
    double z_vt =
        xPrime * Math.sin(Math.toRadians(visionYa)) + zPrime * Math.cos(Math.toRadians(visionYa));
    // Should be offset variables and change based of camera location relative to the center of the
    // robot
    double deltaRobotX = -(detection.x + x_vt - 5);
    double deltaRobotY = -(detection.z + z_vt - 14);

    // double deltaRobotX = -(detection.x + x_vt - Constants.VISION_TOTE_CAM_OFFSET[0]);
    // double deltaRobotY = -(detection.z + z_vt - Constants.VISION_TOTE_CAM_OFFSET[1]);

    double botRadians = Units.degreesToRadians(m_dts.getPose().getRotation().getDegrees());
    double deltaFieldX = deltaRobotX * Math.cos(botRadians) - deltaRobotY * Math.sin(botRadians);
    double deltaFieldY = deltaRobotX * Math.sin(botRadians) + deltaRobotY * Math.cos(botRadians);
    double finalAngle = visionYa + finalYa + Units.radiansToDegrees(botRadians);

    return new Pose2d(
        Units.inchesToMeters(deltaFieldX),
        Units.inchesToMeters(deltaFieldY),
        new Rotation2d(Units.degreesToRadians(finalAngle)));
  }

  private void updateDashboard() {
    SmartDashboard.putNumber("xPidTarget", m_x_target);
    SmartDashboard.putNumber("yPidTarget", m_y_target);
    SmartDashboard.putNumber("rotPidTarget", m_rot_target);
    SmartDashboard.putNumber("xPrime", m_xPrime);
    SmartDashboard.putNumber("zPrime", m_zPrime);
    SmartDashboard.putNumber("finalYa", m_finalYa);
    SmartDashboard.putNumber("m_tagId", m_tagID);
    SmartDashboard.putNumber("xPidStart", m_x_start);
    SmartDashboard.putNumber("yPidStart", m_y_start);
    SmartDashboard.putNumber("rotPidStart", m_rot_start);
    SmartDashboard.putBoolean("isVisionAuto", isVisionAuto);
    SmartDashboard.putNumber("pidXPose", m_dts.getPose().getX());
    SmartDashboard.putNumber("pidYPose", m_dts.getPose().getY());
  }
}
