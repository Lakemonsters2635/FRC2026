// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AgitateCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.IntakeAgitatorCommand;
import frc.robot.commands.IntakeAngleCommand;
import frc.robot.commands.IntakeAngleUpCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeOutCommand;
import frc.robot.commands.ManualTurret;
import frc.robot.commands.PureShoot;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.UptakeCommand;
import frc.robot.commands.UptakeReverseCommand;
import frc.robot.subsystems.ActuatorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeAngleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ObjectTrackerSubsystem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.UptakeSubsystem;
import frc.robot.subsystems.VectorWheelSubsystem;

public class RobotContainer {
  private boolean forward = true;
  private static final int SHAKE_INTERVAL_MS = 500;
  private long lastFlipTime = 0;
  private final Timer shootTimer = new Timer();

  private boolean aimingAprilTag = true;

  // Joysticks
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  public static Joystick rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_CHANNEL);
  public static Joystick leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_CHANNEL);

  // SUBSYSTEMS
  private static ObjectTrackerSubsystem m_objectTrackerSubsystem =
      new ObjectTrackerSubsystem("shripFront");
  private static ActuatorSubsystem m_actuatorSubsystem =
      new ActuatorSubsystem(m_objectTrackerSubsystem);
  private static ShooterSubsystem m_shooterSubsystem =
      new ShooterSubsystem(m_objectTrackerSubsystem);
  private static TurretSubsystem m_turretSubsystem = new TurretSubsystem(m_objectTrackerSubsystem);
  static DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private static IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private static RollerSubsystem m_rollerSubsystem = new RollerSubsystem();
  private static UptakeSubsystem m_uptakeSubsystem = new UptakeSubsystem();
  private static VectorWheelSubsystem m_vectorWheelSubsystem = new VectorWheelSubsystem();
  private static IntakeAngleSubsystem m_intakeAngleSubsystem = new IntakeAngleSubsystem();

  // Cmmands
  private static AgitateCommand m_agitateCommand = new AgitateCommand(m_vectorWheelSubsystem);
  private static IntakeAgitatorCommand m_intakeAgitatorCommand =
      new IntakeAgitatorCommand(m_intakeAngleSubsystem);
  private static UptakeCommand m_uptakeCommand = new UptakeCommand(m_uptakeSubsystem);
  private static UptakeReverseCommand m_uptakeReverseCommand =
      new UptakeReverseCommand(m_uptakeSubsystem);
  private static ShooterCommand m_shooterCommand = new ShooterCommand(m_shooterSubsystem);
  private static IntakeCommand m_intakeCommand =
      new IntakeCommand(m_intakeSubsystem, m_intakeAngleSubsystem);
  private static IntakeOutCommand m_intakeOutCommand =
      new IntakeOutCommand(m_intakeSubsystem, m_intakeAngleSubsystem);
  private static ManualTurret m_manualTurret = new ManualTurret(m_turretSubsystem);
  private static IntakeAngleCommand m_intakeAngleCommand =
      new IntakeAngleCommand(m_intakeAngleSubsystem);
  private static Autos m_autos =
      new Autos(
          m_drivetrainSubsystem,
          m_objectTrackerSubsystem,
          m_rollerSubsystem,
          m_uptakeSubsystem,
          m_vectorWheelSubsystem,
          m_shooterSubsystem,
          m_turretSubsystem,
          m_intakeAngleSubsystem,
          m_actuatorSubsystem,
          m_intakeSubsystem);

  public RobotContainer() {
    // autoChooser.setDefaultOption(
    //     "Leave", new PidAutoCommand(m_drivetrainSubsystem, m_objectTrackerSubsystem, 0, 1, 0));
    // autoChooser.addOption(
    //     "Shoot", new PidAutoCommand(m_drivetrainSubsystem, m_objectTrackerSubsystem, 0, 1, 0));
    // NOT USED!!!!!
    autoChooser.addOption("LeftDepo", m_autos.leftDepoAuto());
    autoChooser.addOption("RightShoot", m_autos.rightShootAuto());
    // autoChooser.addOption(
    //     "LeftShootRamp", m_autos.leftShootRampAuto());
    autoChooser.setDefaultOption("Mid", m_autos.middleScoreAuto());
    autoChooser.addOption("MidDepo", m_autos.midToDepoAuto());
    SmartDashboard.putData("Auto Mode", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    // left joystick buttons
    Trigger shootButton = new JoystickButton(leftJoystick, 1);
    Trigger shootFromMidButton = new JoystickButton(leftJoystick, 3);
    Trigger hexAndRollerOutButton = new JoystickButton(leftJoystick, 4);
    Trigger lockTurret = new JoystickButton(leftJoystick, 5);
    Trigger upShooterVoltageButton = new JoystickButton(leftJoystick, 10);
    Trigger intakeAngleDown = new JoystickButton(leftJoystick, 11);

    // Trigger throttleControl = new JoystickButton(leftJoystick, 4);
    // Trigger moveTurretLeft = new JoystickButton(leftJoystick, 11);
    // Trigger moveTurretRight = new JoystickButton(leftJoystick, 6);
    // Trigger aimTurretAtAprilTag = new JoystickButton(leftJoystick, 7);
    // Trigger downShooterVoltageButton = new JoystickButton(leftJoystick, 12);
    // right joystick button
    Trigger intakeInward = new JoystickButton(rightJoystick, 1);
    Trigger intakeAgitatorButton = new JoystickButton(rightJoystick, 2);
    Trigger intakeBoostButton = new JoystickButton(rightJoystick, 3);
    Trigger hexAndRollerInButton = new JoystickButton(rightJoystick, 14);
    Trigger intakeOutward = new JoystickButton(rightJoystick, 4);

    Trigger uptakeReverseButton = new JoystickButton(rightJoystick, 6);
    Trigger rollerShakeButton = new JoystickButton(rightJoystick, 7);
    Trigger swerveResetButton = new JoystickButton(rightJoystick, 9);
    Trigger intakeUpButton = new JoystickButton(rightJoystick, 11);

    // Trigger vectorWheelInButton = new JoystickButton(rightJoystick, 8);
    // Trigger rollerOutButton = new JoystickButton(rightJoystick, 3);

    // Trigger rollerInButton = new JoystickButton(rightJoystick, 5);
    // Trigger vectorWheelOutButton = new JoystickButton(rightJoystick, 2);
    intakeBoostButton.onTrue(new InstantCommand(() -> m_intakeSubsystem.setBoolHighPower(true)));
    intakeBoostButton.onFalse(new InstantCommand(() -> m_intakeSubsystem.setBoolHighPower(false)));
    intakeUpButton.onTrue(new IntakeAngleUpCommand(m_intakeAngleSubsystem));
    upShooterVoltageButton.onTrue(
        new InstantCommand(() -> m_shooterSubsystem.deltaShooterVoltage(0.3)));
    intakeAgitatorButton.whileTrue(m_intakeAgitatorCommand);
    // downShooterVoltageButton.onTrue(new
    // InstantCommand(()->m_shooterSubsystem.deltaShooterVoltage(-0.3)));

    uptakeReverseButton.whileTrue(m_uptakeReverseCommand);
    hexAndRollerInButton.whileTrue(
        new ParallelCommandGroup(
            new StartEndCommand(
                () -> m_vectorWheelSubsystem.setVectorWheelsIn(),
                () -> m_vectorWheelSubsystem.stopVectorWheels(),
                m_vectorWheelSubsystem),
            new StartEndCommand(
                () -> m_rollerSubsystem.setRollersForward(),
                () -> m_rollerSubsystem.stopRollers(),
                m_rollerSubsystem)));

    hexAndRollerOutButton.whileTrue(
        new ParallelCommandGroup(
            new StartEndCommand(
                () -> m_vectorWheelSubsystem.setVectorWheelsOut(),
                () -> m_vectorWheelSubsystem.stopVectorWheels(),
                m_vectorWheelSubsystem),
            new StartEndCommand(
                () -> m_rollerSubsystem.setRollersBackward(),
                () -> m_rollerSubsystem.stopRollers(),
                m_rollerSubsystem)));

    shootFromMidButton.whileFalse(
        new SequentialCommandGroup(
            new InstantCommand(() -> aimingAprilTag = true).withTimeout(.01),
            new InstantCommand(() -> m_actuatorSubsystem.setAutoControl(true)).withTimeout(.01),
            new InstantCommand(() -> m_turretSubsystem.setMidMode(false)).withTimeout(.01)));

    shootFromMidButton.whileTrue(
        new SequentialCommandGroup(
            new InstantCommand(() -> aimingAprilTag = false).withTimeout(.01),
            new InstantCommand(() -> m_actuatorSubsystem.setAutoControl(false)).withTimeout(.01),
            new InstantCommand(() -> m_turretSubsystem.setMidMode(true)).withTimeout(.01),
            new ParallelCommandGroup(
                new InstantCommand(() -> m_actuatorSubsystem.setPosition(0.6)),
                new Shoot(
                    m_uptakeSubsystem,
                    m_rollerSubsystem,
                    m_shooterSubsystem,
                    m_vectorWheelSubsystem,
                    m_actuatorSubsystem,
                    false))));

    // shootFromMidButton.whileTrue(
    //     new SequentialCommandGroup(
    //         new SetLinearPoseCommand(m_actuatorSubsystem, 0.6).withTimeout(0.01),
    //         new InstantCommand(() -> m_actuatorSubsystem.setAutoControl(false)).withTimeout(.01),
    //         new InstantCommand(() -> m_turretSubsystem.setMidMode(true)).withTimeout(.01),
    //         new Shoot(
    //             m_uptakeSubsystem,
    //             m_rollerSubsystem,
    //             m_shooterSubsystem,
    //             m_vectorWheelSubsystem,
    //             false),
    //         new InstantCommand(() -> m_actuatorSubsystem.setAutoControl(true)).withTimeout(.01),
    //         new InstantCommand(() -> m_turretSubsystem.setMidMode(false)).withTimeout(.01)));

    swerveResetButton.onTrue(
        new SequentialCommandGroup(
            new InstantCommand(() -> m_drivetrainSubsystem.resetAngle(0)).withTimeout(0.01),
            new InstantCommand(() -> m_drivetrainSubsystem.zeroOdometry()).withTimeout(.01)));

    // vectorWheelOutButton.whileTrue(
    //     new StartEndCommand(
    //         () -> m_vectorWheelSubsystem.setVectorWheelsOut(),
    //         () -> m_vectorWheelSubsystem.stopVectorWheels(),
    //         m_vectorWheelSubsystem));

    // rollerOutButton.whileTrue(
    //     new StartEndCommand(
    //         () -> m_rollerSubsystem.setRollersBackward(),
    //         () -> m_rollerSubsystem.stopRollers(),
    //         m_rollerSubsystem));

    // rollerInButton.whileTrue(
    //     new StartEndCommand(
    //         () -> m_rollerSubsystem.setRollersForward(),
    //         () -> m_rollerSubsystem.stopRollers(),
    //         m_rollerSubsystem));

    intakeAngleDown.whileTrue(
        new SequentialCommandGroup(
            new IntakeAngleCommand(m_intakeAngleSubsystem), // runs for 0.s
            new RunCommand(
                () -> m_intakeAngleSubsystem.intakeAngleFeedForward(), m_intakeAngleSubsystem)));

    rollerShakeButton.whileTrue(
        new StartEndCommand(
                // onStart
                () -> {
                  lastFlipTime = System.currentTimeMillis();
                },

                // onEnd
                () -> m_rollerSubsystem.stopRollers(),
                m_rollerSubsystem)
            .andThen(
                new RunCommand(
                    () -> {
                      long now = System.currentTimeMillis();
                      if (now - lastFlipTime > SHAKE_INTERVAL_MS) {
                        lastFlipTime = now;
                        forward = !forward;

                        if (forward) {
                          m_rollerSubsystem.setRollersForward();
                        } else {
                          m_rollerSubsystem.setRollersBackward();
                        }
                      }
                    },
                    m_rollerSubsystem)));

    lockTurret.toggleOnTrue(
        new SequentialCommandGroup(
            new InstantCommand(() -> m_turretSubsystem.setDriverControl(true)),
            new InstantCommand(() -> m_turretSubsystem.setAutoControl(false)),
            new InstantCommand(() -> m_turretSubsystem.setTurretTarget(0))));
    lockTurret.toggleOnFalse(new InstantCommand(() -> m_turretSubsystem.setDriverControl(false)));
    double antiJamTimeout = 0.2;
    double shootingTimeout = 1.8;
    shootButton.whileTrue(
        new SequentialCommandGroup(
            new PureShoot(m_shooterSubsystem).withTimeout(.2),
            new Shoot(
                m_uptakeSubsystem,
                m_rollerSubsystem,
                m_shooterSubsystem,
                m_vectorWheelSubsystem,
                m_actuatorSubsystem,
                true)));

    // shootButton.whileTrue(
    //     new RunCommand(
    //             () -> {
    //               // Start timer on first loop
    //               if (!shootTimer.isRunning()) {
    //                 shootTimer.reset();
    //                 shootTimer.start();
    //               }

    //               // Always run shooter
    //               m_shooterSubsystem.shoot();
    //               m_rollerSubsystem.setRollersForward();

    //               // After 0.5 seconds, run uptake
    //               if (shootTimer.hasElapsed(0.5)) {
    //                 m_uptakeSubsystem.uptake();
    //                 m_vectorWheelSubsystem.setVectorWheelsIn();
    //               }
    //               if (System.currentTimeMillis() % 1000 > 500) {
    //                 m_rollerSubsystem.setRollersForward();
    //               } else {
    //                 m_rollerSubsystem.setRollersBackward();
    //               }
    //             },
    //             m_shooterSubsystem,
    //             m_uptakeSubsystem,
    //             m_vectorWheelSubsystem)
    //         .finallyDo(
    //             interrupted -> {
    //               shootTimer.stop();

    //               m_shooterSubsystem.shooterStop();
    //               m_uptakeSubsystem.stopUptake();
    //               m_vectorWheelSubsystem.stopVectorWheels();
    //               m_rollerSubsystem.stopRollers();
    //             }));

    // shootButton.whileTrue(new SequentialCommandGroup(
    //   m_shooterCommand.withTimeout(0),
    //   m_uptakeCommand
    // ));

    // manualTurredButton.onTrue(m_manualTurret);
    intakeInward.whileTrue(m_intakeCommand);

    intakeOutward.whileTrue(m_intakeOutCommand);

    // throttleControl.whileTrue(
    //     new InstantCommand(
    //         () ->
    //             CommandScheduler.getInstance()
    //                 .schedule(
    //                     new SetLinearPoseCommand(
    //                         m_actuatorSubsystem,
    //                         MathUtil.clamp(Math.abs(leftJoystick.getThrottle()), .24, .8)))));

    // right joystick buttons

    // turretButton.whileTrue(new InstantCommand(()->m_turretSubsystem.turretPower(1)));
    // oppositeTurretButton.whileTrue(new InstantCommand(()->m_turretSubsystem.turretPower(-1)));

    // vectorWheelInButton.whileTrue(m_agitateCommand);
    // uptakeButton.whileTrue(m_uptakeCommand);
    // moveTurretLeft.onTrue(new InstantCommand(() -> m_turretSubsystem.moveTurretLeft()));
    // moveTurretRight.onTrue(new InstantCommand(() -> m_turretSubsystem.moveTurretRight()));
    // Need to change this depending on the alliance, only for testing, be carefull
    // aimTurretAtAprilTag.onTrue(new InstantCommand(() -> m_turretSubsystem.aimAtTarget(10)));
  }

  public Command getAutonomousCommand() {
    // return m_autos.leftDepoAuto();
    // return autoChooser.getSelected();
    return m_autos.midToDepoAuto();
    // return m_autos.leftShootAuto();
    // return m_autos.middleScoreAuto();
    // return m_autos.rightShootAuto();
    // return m_autos.test();
  }
}
