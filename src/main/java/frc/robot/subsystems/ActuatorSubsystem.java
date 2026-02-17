// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ActuatorSubsystem extends SubsystemBase {
  /** Creates a new ActuatorSubsystem. */
  //LinearServo m_linearActuatorLeft;
  //LinearServo m_linearActuatorRight;
  Servo m_linearActuatorRight;
  Servo m_linearActuatorLeft;
  public ActuatorSubsystem() {
    m_linearActuatorLeft = new Servo(Constants.LEFT_ACTUATOR_ID);
    m_linearActuatorRight = new Servo(Constants.RIGHT_ACTUATOR_ID);
  }

  public void setPosition(double pos){ 
    //pos in between 0-1 
    //in reality the actuator only moves between 0.22 <= x <= .8
    //m_linearActuatorLeft.setNewPosition(pos);
    //m_linearActuatorRight.setNewPosition(pos);
    m_linearActuatorLeft.setPosition(pos);
    m_linearActuatorRight.setPosition(pos);
  }

  public double getPosition(){
    return m_linearActuatorLeft.getPosition();
  }
  

  public boolean atTarget(double targetPose){
    if(Math.abs(m_linearActuatorLeft.getPosition()-targetPose) < 0.03 && Math.abs(m_linearActuatorRight.getPosition()-targetPose) < 0.03){
      return true;
    }
    return false;
  }
  
  @Override
  public void periodic() {
    //m_linearActuatorLeft.updateCurPos();
   // m_linearActuatorRight.updateCurPos();
    SmartDashboard.putNumber("getPosActuatorLeft", getPosition());
    SmartDashboard.putNumber("getPosActuatorRight", m_linearActuatorRight.getPosition());
    // SmartDashboard.putNumber("getPosActuatorRight", m_linearActuatorRight.getEstimatedPosition());
    setPosition(MathUtil.clamp(Math.abs(RobotContainer.leftJoystick.getThrottle()),.24,.65)); //.65 is hard stop
    SmartDashboard.putNumber("throttle",Math.abs(RobotContainer.leftJoystick.getThrottle()));
    // This method will be called once per scheduler run
  }
}
