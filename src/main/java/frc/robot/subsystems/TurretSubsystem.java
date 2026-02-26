// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.REVLibError;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class TurretSubsystem extends SubsystemBase {
  /** Creates a new TurretSubsystem. */
  private final SparkMax m_turretSparkMax;
  PIDController m_turretController;
  double pid;
  public double m_poseTarget;

  public TurretSubsystem() {
     m_turretSparkMax = new SparkMax(Constants.TURRET_MOTOR_ID, MotorType.kBrushless); 
     m_turretSparkMax.getEncoder().setPosition(0);
     m_turretController = new PIDController(0.08,0,0); // TODO: change values
     
  }

  public void turretPower(double power){
    m_turretSparkMax.setVoltage(power);
  }

  public void resetEncoder(){
    m_turretSparkMax.getEncoder().setPosition(0);
  }

  public void stopTurret(){
    m_turretSparkMax.setVoltage(0);
  }

  public double getEncoder(){
    return m_turretSparkMax.getEncoder().getPosition();
  }

  public double getDegrees(){
    double deg = (getEncoder()/(Constants.RATIO_SPARKMAX_ROTATION_TO_TURRET * Constants.ENCODER_TICS_PER_SPARKMAX_REVOLUTION)) * 360;
    return deg % 360;
  }

  public void moveTurretLeft(){
    setTurretTarget(m_poseTarget-Constants.TURRET_ANGLE_MOVE); // do not know if this is correct (+/-)
  }

  public void moveTurretRight(){
    setTurretTarget(m_poseTarget+Constants.TURRET_ANGLE_MOVE); // do not know if this is correct (+/-)
  }

  public void setTurretTarget(double position){
    m_poseTarget = position;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Encoder Counts", getEncoder());
    SmartDashboard.putNumber("Turret Degrees", getDegrees());
    pid = MathUtil.clamp(m_turretController.calculate(getDegrees(), m_poseTarget), -Constants.TURRET_POWER, Constants.TURRET_POWER);
    turretPower(pid);
    // This method will be called once per scheduler run
  }
}
