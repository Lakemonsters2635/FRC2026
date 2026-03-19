// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  ObjectTrackerSubsystem m_objectTrackerSubsystem;

  boolean isMidMode = false;
  TalonFX m_shooterMotorLeft;
  
  TalonFX m_shooterMotorRight;
  VoltageConfigs m_leftConfig;
  VoltageConfigs m_rightConfig;

  TalonFX motor = new TalonFX(1);
VelocityVoltage velocityRequest = new VelocityVoltage(0);

  Joystick joystick = new Joystick(0);
  double power = 7;
  double savePower;
  double magnitude;
  public ShooterSubsystem(ObjectTrackerSubsystem objectTrackerSubsystem) {
    m_leftConfig = new VoltageConfigs();
    m_rightConfig = new VoltageConfigs();
    m_rightConfig.SupplyVoltageTimeConstant = 10;
    m_leftConfig.SupplyVoltageTimeConstant = 10;


    m_objectTrackerSubsystem = objectTrackerSubsystem;
    m_shooterMotorLeft = new TalonFX(Constants.SHOOTER_MOTOR_ID_LEFT);
    m_shooterMotorRight = new TalonFX(Constants.SHOOTER_MOTOR_ID_RIGHT);

    m_shooterMotorLeft.getConfigurator().apply(m_leftConfig);
    m_shooterMotorRight.getConfigurator().apply(m_rightConfig);

     setPID(Constants.SHOOTER_P, Constants.SHOOTER_I, Constants.SHOOTER_D);
    // SmartDashboard.putNumber("Shooter Power", 8);
  }

  public void shoot() {
    Pose2d target = m_objectTrackerSubsystem.getNearestAprilTagDistShooter();

    magnitude = Math.sqrt(target.getX() * target.getX() + target.getY() * target.getY());
    // if (magnitude != 0) {
    //   double c = magnitude < 2 ? -0.2 : 0;
    //   savePower = ((power) + (magnitude / 4.35));
    
    //   m_shooterMotorRight.setVoltage(
    //       ((power) + (magnitude / 4.35 * SmartDashboard.getNumber("a", 1.1)-SmartDashboard.getNumber("b", 0.2)+c)) 
    //           * -1); // SmartDashboard.getNumber("Shooter Power", 8) * -1);
    //   m_shooterMotorLeft.setVoltage(
    //       ((power) + (magnitude / 4.35 * SmartDashboard.getNumber("a", 1.1)-SmartDashboard.getNumber("b", 0.2)+c)));  // SmartDashboard.getNumber("Shooter Power", 8);
    //   SmartDashboard.putNumber("Shooter Volt", (power + (magnitude / 4.35)));
    // }
    // else{
    //   m_shooterMotorRight.setVoltage(savePower* -1); 
    //   m_shooterMotorLeft.setVoltage(savePower);
    // }

    //  if (magnitude != 0) {
    //   savePower = ((power) + (magnitude / 4.35));
    //   m_shooterMotorRight.setVoltage(
    //       ((power) + (magnitude / 4.35 * SmartDashboard.getNumber("a", 1.1)-SmartDashboard.getNumber("b", 0.2))) 
    //           * -1); // SmartDashboard.getNumber("Shooter Power", 8) * -1);
    //   m_shooterMotorLeft.setVoltage(
    //       ((power) + (magnitude / 4.35)));  // SmartDashboard.getNumber("Shooter Power", 8);
    //   SmartDashboard.putNumber("Shooter Volt", (power + (magnitude / 4.35)));
    // }
    // else{
    //   m_shooterMotorRight.setVoltage(savePower* -1); 
    //   m_shooterMotorLeft.setVoltage(savePower);
    // }
    if (magnitude != 0) {
      if(magnitude > 2.8){
        savePower = ((power) + (magnitude / 4.35));
        m_shooterMotorRight.setVoltage(
            ((power) + (magnitude / 4.35)) 
                * -1); // SmartDashboard.getNumber("Shooter Power", 8) * -1);
                m_shooterMotorLeft.setc
        m_shooterMotorLeft.setVoltage(
            ((power) + (magnitude / 4.35)));  // SmartDashboard.getNumber("Shooter Power", 8);
        SmartDashboard.putNumber("Shooter Volt", (power + (magnitude / 4.35)));
        SmartDashboard.putNumber("shootNum", 1.1);
      }
      else if(magnitude > 1.3){
        savePower = ((power) + (magnitude / 4.65));
        m_shooterMotorRight.setVoltage(
            ((power) + (magnitude / 4.60))
                * -1); // SmartDashboard.getNumber("Shooter Power", 8) * -1);
        m_shooterMotorLeft.setVoltage(
            ((power) + (magnitude / 4.60)));  // SmartDashboard.getNumber("Shooter Power", 8);
        SmartDashboard.putNumber("Shooter Volt", (power + (magnitude / 4.6)));
        SmartDashboard.putNumber("shootNum", 1.2);

      }
      else{
        m_shooterMotorLeft.setVoltage(6.5);
        m_shooterMotorRight.setVoltage(-6.5);
        SmartDashboard.putNumber("shootNum", 1.3);

      }
    }
    else{
      if(savePower!= 0){
        m_shooterMotorRight.setVoltage(savePower* -1); 
        m_shooterMotorLeft.setVoltage(savePower);
        SmartDashboard.putNumber("shootNum", 2.1);

      }
      else{
        m_shooterMotorLeft.setVoltage(6.5);
        m_shooterMotorRight.setVoltage(-6.5);        
        SmartDashboard.putNumber("shootNum", 2.2);

      }
      
    }
  }

  public void setPID(double p, double i, double d) {
    slot0Configs.kP = p;
    slot0Configs.kI = i;
    slot0Configs.kD = d;
    outTakeMotor.getConfigurator().apply(slot0Configs);
  }

  
  public void velocityController() {
    motor.setControl(velocityRequest.withVelocity(50));
  }

  public void shootFar() {
    m_shooterMotorRight.setVoltage(
        Constants.SHOOTER_FAR_POWER * -1); // SmartDashboard.getNumber("Shooter Power", 8) * -1);
    m_shooterMotorLeft.setVoltage(
        Constants.SHOOTER_FAR_POWER); // SmartDashboard.getNumber("Shooter Power", 8);
    // SmartDashboard.putNumber("Shooter Volt", (power + (magnitude / 4.35)));
  }

  public void shooterStop() {
    m_shooterMotorLeft.setVoltage(Constants.MOTOR_STOP);
    m_shooterMotorRight.setVoltage(Constants.MOTOR_STOP);
  }

  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("mag of dist center camera to center hub", magnitude);
    // power = (joystick.getThrottle() + 1) * 5;
    // This method will be called once per scheduler run
  }
}

