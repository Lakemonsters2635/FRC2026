// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class IntakeAngleSubsystem extends SubsystemBase {
  /** Creates a new IntakeAngleSubsystem. */
  TalonFX intakeAngleMotor;
  // Slot1Configs m_slot1Configs = new Slot1Configs();
  VoltageConfigs m_config;
  // PositionVoltage m_posController;
  double m_targetPos;
  PIDController m_pidController = new PIDController(0.02,0.001,0);
  double ff = 0;
  double initialPos;
  boolean pidMode= false;

  public IntakeAngleSubsystem() {
    intakeAngleMotor = new TalonFX(11);
    intakeAngleMotor.setNeutralMode(NeutralModeValue.Brake);

    m_config = new VoltageConfigs();
    m_config.SupplyVoltageTimeConstant = 10;
    intakeAngleMotor.getConfigurator().apply(m_config);

    initialPos = intakeAngleMotor.getPosition().getValueAsDouble();
    m_targetPos = getAngle();


    // intakeAngleMotor.getConfigurator().apply(m_config);

    // m_posController = new PositionVoltage(m_targetPos);

    // posController(m_targetPos);

    // SmartDashboard.putNumber("InAngle P", 0);
    // SmartDashboard.putNumber("InAngle I", 0);
    // SmartDashboard.putNumber("InAngle D", 0);
    // SmartDashboard.putNumber("InAngle ff", -2);
    // SmartDashboard.putNumber("InAngle target", m_targetPos);


    // SmartDashboard.putNumber("Intake Angle Feed Forward voltage", -1.5);
  }

  public void chagePidMode(boolean newMode){
    pidMode = newMode;
  }

  public double getAngle(){
    // Range should be 0 to 90 apprx
    return -1 * (intakeAngleMotor.getPosition().getValueAsDouble() - initialPos) / (6.578128) * 360;
  }

  // public void posController(double pos){
  //   double temp = pos;
  //   intakeAngleMotor.setControl(
  //     new PositionDutyCycle(pos)//.withFeedForward(-2)
  //   );
  // }

  // public void setPIDValues(double p, double i, double d, double ff){
  //   m_slot1Configs.kP = p;
  //   m_slot1Configs.kI = i;
  //   m_slot1Configs.kD = d;
  //   m_slot1Configs.kG = ff;

  //   intakeAngleMotor.getConfigurator().apply(m_slot1Configs);
  // }

  public void intakeAngleDown() {
    // not tested
    intakeAngleMotor.setVoltage(0.5); // +
  }

  public void intakeBarelyUp(){
    intakeAngleMotor.setVoltage(RobotContainer.leftJoystick.getThrottle()*2);
  }

   public void intakeAngleUp() {
    // not tested
    intakeAngleMotor.setVoltage(-2.5); // +
  }

  public void intakeAngleDownHard() {
    // not tested
    intakeAngleMotor.setVoltage(1); // +
  }

  public void setVolts(double volts) {
    intakeAngleMotor.setVoltage(volts);
  }

  public void intakeAngleFeedForward() {
    intakeAngleMotor.setVoltage(-.25); // working, but change for better results
  }

  public void intakeAngleStop() {
    intakeAngleMotor.setVoltage(0); // -
  }

  public void setTargetPos(double target){
    m_targetPos = target;
  }

  @Override
  public void periodic() {
    // setPIDValues(
      SmartDashboard.getNumber("InAngle P", 0);
      SmartDashboard.getNumber("InAngle I", 0);
      SmartDashboard.getNumber("InAngle D", 0); 
      SmartDashboard.getNumber("InAngle ff", -2);
    // );
    // posController(SmartDashboard.getNumber("InAngle target", m_targetPos));
    SmartDashboard.putNumber("InAngle curr angle", getAngle());

    if(pidMode){
      double fb = m_pidController.calculate(getAngle(), m_targetPos);
      ff = -2.5;//-2.36;
      // setVolts(ff * Math.cos(getAngle() * Math.PI/180));
      setVolts(ff * Math.cos(getAngle() * Math.PI/180) - fb);
    }
  }
}
