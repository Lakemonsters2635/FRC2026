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

public class PidRotationAutoCommand extends Command {

  private final DrivetrainSubsystem dts;
  private final ObjectTrackerSubsystem ots;

  private final boolean useVision;
  private final int tagId;

  private double targetX;
  private double targetY;
  private double targetRotDeg;

  private double startX, startY;

  private final PIDController pidX = new PIDController(4.5, 0, 0.1);
  private final PIDController pidY = new PIDController(4.5, 0, 0.1);
  private final PIDController pidRot = new PIDController(2, 0, 0.2);

  private static final double MAX_SPEED = 3.5;
  private static final double MAX_ROT_SPEED = Math.PI;

  private final Trigger cancelTrigger =
      new JoystickButton(RobotContainer.rightJoystick, 6);

  public PidRotationAutoCommand(
      DrivetrainSubsystem dts,
      ObjectTrackerSubsystem ots,
      int tagId,
      double xPrime,
      double zPrime,
      double finalYa,
      boolean useVision) {

    this.dts = dts;
    this.ots = ots;
    this.tagId = tagId;
    this.useVision = useVision;

    Pose2d targetPose =
        useVision ? computeVisionTarget(xPrime, zPrime, finalYa, tagId)
                  : dts.getPose();

    targetX = targetPose.getX();
    targetY = targetPose.getY();
    targetRotDeg = targetPose.getRotation().getDegrees();

    addRequirements(dts, ots);
  }

  // Manual constructor (no vision)
  public PidRotationAutoCommand(
      DrivetrainSubsystem dts,
      ObjectTrackerSubsystem ots,
      double xTarget,
      double yTarget,
      double rotTargetDeg) {

    this(dts, ots, -1, 0, 0, 0, false);
    this.targetX = xTarget;
    this.targetY = yTarget;
    this.targetRotDeg = rotTargetDeg;
  }

  @Override
  public void initialize() {
    dts.stashAngle();
    dts.resetAngle();
    dts.zeroOdometry();

    startX = dts.getPose().getX();
    startY = dts.getPose().getY();

    pidRot.enableContinuousInput(-180, 180);

    dts.setFollowJoystick(false);
    dts.stopMotors();
  }

  @Override
  public void execute() {
    Pose2d pose = dts.getPose();
    double x = pose.getX();
    double y = pose.getY();
    double rotDeg = pose.getRotation().getDegrees();

    // Compute PID outputs
    double vx = Math.abs(targetX - x) > 0.03 ? pidX.calculate(x, targetX) : 0;
    double vy = Math.abs(targetY - y) > 0.03 ? pidY.calculate(y, targetY) : 0;

    double vRot =
        Math.abs(targetRotDeg - rotDeg) > 2
            ? -pidRot.calculate(rotDeg, targetRotDeg)
            : 0;

    // Clamp speeds
    double mag = Math.hypot(vx, vy);
    if (mag > MAX_SPEED) {
        vx = vx / mag * MAX_SPEED;
        vy = vy / mag * MAX_SPEED;
    }

    vRot = MathUtil.clamp(vRot, -MAX_ROT_SPEED, MAX_ROT_SPEED);

    dts.drive(vx, vy, vRot, true);


    updateDashboard(vx, vy, vRot);
  }

  @Override
  public void end(boolean interrupted) {
    dts.restoreAngle();
    dts.stopMotors();
    dts.setFollowJoystick(true);
  }

  @Override
  public boolean isFinished() {
    if (cancelTrigger.getAsBoolean()) return true;

    Pose2d pose = dts.getPose();
    double x = pose.getX();
    double y = pose.getY();
    double rot = pose.getRotation().getDegrees();

    return Math.abs(targetX - x) < 0.05
        && Math.abs(targetY - y) < 0.05
        && Math.abs(targetRotDeg - rot) < 2;
  }

  private Pose2d computeVisionTarget(
      double xPrime, double zPrime, double finalYa, int tagId) {

    Detection det = ots.getSpecificAprilTag(tagId);
    if (det == null) return new Pose2d();

    double visionYa = -det.ya;

    double x_vt =
        xPrime * Math.cos(Math.toRadians(visionYa))
            - zPrime * Math.sin(Math.toRadians(visionYa));

    double z_vt =
        xPrime * Math.sin(Math.toRadians(visionYa))
            + zPrime * Math.cos(Math.toRadians(visionYa));

    double deltaRobotX = -(det.x + x_vt - 5);
    double deltaRobotY = -(det.z + z_vt - 14);

    double botRad = dts.getPose().getRotation().getRadians();

    double fieldX = deltaRobotX * Math.cos(botRad) - deltaRobotY * Math.sin(botRad);
    double fieldY = deltaRobotX * Math.sin(botRad) + deltaRobotY * Math.cos(botRad);

    double finalAngle = visionYa + finalYa + Math.toDegrees(botRad);

    return new Pose2d(
        Units.inchesToMeters(fieldX),
        Units.inchesToMeters(fieldY),
        Rotation2d.fromDegrees(finalAngle));
  }

  private void updateDashboard(double vx, double vy, double vRot) {
    SmartDashboard.putNumber("PID Target X", targetX);
    SmartDashboard.putNumber("PID Target Y", targetY);
    SmartDashboard.putNumber("PID Target Rot", targetRotDeg);

    SmartDashboard.putNumber("PID Out X", vx);
    SmartDashboard.putNumber("PID Out Y", vy);
    SmartDashboard.putNumber("PID Out Rot", vRot);

    SmartDashboard.putBoolean("Using Vision", useVision);
  }
}