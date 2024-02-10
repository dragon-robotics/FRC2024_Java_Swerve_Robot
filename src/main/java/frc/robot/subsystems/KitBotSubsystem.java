// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KitBotConstants;

public class KitBotSubsystem extends SubsystemBase {
  /** Creates a new KitBotSubsystem. */
  private CANSparkMax m_motorLeadA = new CANSparkMax(KitBotConstants.KITBOT_LEADER_ID_A, MotorType.kBrushless);
  private CANSparkMax m_motorFollowerA = new CANSparkMax(KitBotConstants.KITBOT_FOLLOWER_ID_A, MotorType.kBrushless);

  private CANSparkMax m_motorLeadB = new CANSparkMax(KitBotConstants.KITBOT_LEADER_ID_B, MotorType.kBrushless);
  private CANSparkMax m_motorFollowerB = new CANSparkMax(KitBotConstants.KITBOT_FOLLOWER_ID_B, MotorType.kBrushless);

  private CANSparkMax m_motorArm = new CANSparkMax(10, MotorType.kBrushless);
  
  private SparkPIDController m_pidControllerA;
  private SparkPIDController m_pidControllerB;

  public KitBotSubsystem() {

    m_motorLeadA.restoreFactoryDefaults();
    m_motorFollowerA.restoreFactoryDefaults();

    m_motorLeadB.restoreFactoryDefaults();
    m_motorFollowerB.restoreFactoryDefaults();

    // m_motorFollower.follow(m_motorLead, false);
    m_motorFollowerB.follow(m_motorLeadB, true);


    m_pidControllerA = m_motorLeadA.getPIDController();
    
    m_pidControllerA.setP(KitBotConstants.K_P);
    m_pidControllerA.setI(KitBotConstants.K_I);
    m_pidControllerA.setD(KitBotConstants.K_D);
    m_pidControllerA.setIZone(KitBotConstants.K_IZ);
    m_pidControllerA.setFF(KitBotConstants.K_FF);
    m_pidControllerA.setOutputRange(KitBotConstants.K_MIN_OUTPUT ,KitBotConstants.K_MAX_OUTPUT);

    m_pidControllerB = m_motorLeadB.getPIDController();
    
    m_pidControllerB.setP(KitBotConstants.K_P);
    m_pidControllerB.setI(KitBotConstants.K_I);
    m_pidControllerB.setD(KitBotConstants.K_D);
    m_pidControllerB.setIZone(KitBotConstants.K_IZ);
    m_pidControllerB.setFF(KitBotConstants.K_FF);
    m_pidControllerB.setOutputRange(KitBotConstants.K_MIN_OUTPUT ,KitBotConstants.K_MAX_OUTPUT);
    
    
  }

  public void variableKitBotSpeedA(double yAxisLeft) {
    m_pidControllerA.setOutputRange(-0.69, 0.69);
    m_pidControllerA.setReference(KitBotConstants.MAX_RPM, CANSparkMax.ControlType.kVelocity);
    m_pidControllerA.setSmartMotionMaxVelocity(yAxisLeft, 0);
  }

  public void setAGroup(double speed){
    m_motorLeadA.set(speed);
  }

  public void setBGroup(double speed) {
    m_motorLeadB.set(speed);
  }

  public void shootADummy() {
    m_motorLeadA.set(-0.4);
  }

  public void shootBDummy() {
    m_motorLeadA.set(-0.45);
  }

  public void shootXDummy() {
    m_motorLeadA.set(-0.5);
  }

  public void shootYDummy() {
    m_motorLeadA.set(-0.55);
  }

  public void armExtendoForward() {
    m_motorArm.set(0.25);
  }

  public void armExtendoBackward() {
    m_motorArm.set(-0.25);
  }

  public void stop() {
    m_motorLeadA.set(0);
  }
}
