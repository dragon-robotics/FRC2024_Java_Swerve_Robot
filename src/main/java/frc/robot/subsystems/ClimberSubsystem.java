// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.GeneralConstants.RobotMode;

public class ClimberSubsystem extends SubsystemBase {

  // Create new Shuffleboard tab for the climber subsystem //
  private final ShuffleboardTab m_climberShuffleboardTab = Shuffleboard.getTab("Climber");
  private GenericEntry m_climberLeadPowerPercentageEntry = null;
  private GenericEntry m_climberLeadPowerVoltageEntry = null;
  private GenericEntry m_climberLeadPowerCurrentEntry = null;
  private GenericEntry m_climberLeadPowerTemperatureEntry = null;
  private GenericEntry m_climberLeadPositionEntry = null;
  private GenericEntry m_climberFollowPowerPercentageEntry = null;
  private GenericEntry m_climberFollowPowerVoltageEntry = null;
  private GenericEntry m_climberFollowPowerCurrentEntry = null;
  private GenericEntry m_climberFollowPowerTemperatureEntry = null;
  private GenericEntry m_climberFollowPositionEntry = null;
  
  private final CANSparkMax m_climberLead = new CANSparkMax(ClimberConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax m_climberFollow = new CANSparkMax(ClimberConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
  
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

    /* Lead motor settings */
    // Restore lead motor to factory default //
    m_climberLead.restoreFactoryDefaults();

    // Set the lead motor to only use 10 volts of power //
    m_climberLead.enableVoltageCompensation(ClimberConstants.NOMINAL_VOLTAGE);

    // Set the lead motor to only use 40 amp stall limit //
    m_climberLead.setSmartCurrentLimit(ClimberConstants.STALL_CURRENT_LIMIT);

    // Set the lead motor's hard stop current limit to 60 amps //
    m_climberLead.setSecondaryCurrentLimit(ClimberConstants.SECONDARY_CURRENT_LIMIT);

    // Set the lead motor to brake mode //
    m_climberLead.setIdleMode(IdleMode.kBrake);

    // Enable soft limits for the lead motor //
    m_climberLead.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_climberLead.enableSoftLimit(SoftLimitDirection.kReverse, true);

    // Set the lead motor's soft limit to prevent the climber from going too far down //
    m_climberLead.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.STARTING_LIMIT);
    
    // Set the lead motor's soft limit to prevent the climber from going too far up //
    m_climberLead.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.ENDING_LIMIT);

    /* Follow motor settings */    
    // Restore follow motor to factory default //
    m_climberFollow.restoreFactoryDefaults();

    // Set the follow motor to only use 10 volts of power //
    m_climberFollow.enableVoltageCompensation(ClimberConstants.NOMINAL_VOLTAGE);

    // Set the follow motor to only use 40 amp stall limit //
    m_climberFollow.setSmartCurrentLimit(ClimberConstants.STALL_CURRENT_LIMIT);

    // Set the follow motor's hard stop current limit to 60 amps //
    m_climberFollow.setSecondaryCurrentLimit(ClimberConstants.SECONDARY_CURRENT_LIMIT);

    // Set the follow motor to brake mode //
    m_climberFollow.setIdleMode(IdleMode.kBrake);

    // Enable soft limits for the follow motor //
    m_climberFollow.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_climberFollow.enableSoftLimit(SoftLimitDirection.kReverse, true);

    // Set the follow motor's soft limit to prevent the climber from going too far down //
    m_climberFollow.setSoftLimit(SoftLimitDirection.kReverse, ClimberConstants.STARTING_LIMIT);
    
    // Set the follow motor's soft limit to prevent the climber from going too far up //
    m_climberFollow.setSoftLimit(SoftLimitDirection.kForward, ClimberConstants.ENDING_LIMIT);

    // Set the follow motor to follow the lead motor //
    m_climberFollow.follow(m_climberLead, true);

    // Set the motors initially to 0 speed //
    m_climberLead.set(0);

    // Create Shuffleboard entries for the ClimberSubsystem if the robot is in test mode //
    if (GeneralConstants.CURRENT_MODE == RobotMode.TEST) {
      m_climberLeadPowerPercentageEntry = m_climberShuffleboardTab.add("Climber Lead Power Percentage Reading", m_climberLead.getAppliedOutput()).getEntry();
      m_climberLeadPowerVoltageEntry = m_climberShuffleboardTab.add("Climber Lead Power Voltage Reading", m_climberLead.getBusVoltage()).getEntry();
      m_climberLeadPowerCurrentEntry = m_climberShuffleboardTab.add("Climber Lead Power Current Reading", m_climberLead.getOutputCurrent()).getEntry();
      m_climberLeadPowerTemperatureEntry = m_climberShuffleboardTab.add("Climber Lead Power Temperature Reading", m_climberLead.getMotorTemperature()).getEntry();
      m_climberLeadPositionEntry = m_climberShuffleboardTab.add("Climber Lead Position Reading", m_climberLead.getEncoder().getPosition()).getEntry();

      m_climberFollowPowerPercentageEntry = m_climberShuffleboardTab.add("Climber Follow Power Percentage Reading", m_climberFollow.getAppliedOutput()).getEntry();
      m_climberFollowPowerVoltageEntry = m_climberShuffleboardTab.add("Climber Follow Power Voltage Reading", m_climberFollow.getBusVoltage()).getEntry();
      m_climberFollowPowerCurrentEntry = m_climberShuffleboardTab.add("Climber Follow Power Current Reading", m_climberFollow.getOutputCurrent()).getEntry();
      m_climberFollowPowerTemperatureEntry = m_climberShuffleboardTab.add("Climber Follow Power Temperature Reading", m_climberFollow.getMotorTemperature()).getEntry();
      m_climberFollowPositionEntry = m_climberShuffleboardTab.add("Climber Follow Position Reading", m_climberFollow.getEncoder().getPosition()).getEntry();
    } else {
      // Set status 1-7 to be 50000ms for the lead climber motors
      m_climberLead.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 50000);
      m_climberLead.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50000);
      m_climberLead.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50000);
      m_climberLead.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 50000);
      m_climberLead.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 50000);
      m_climberLead.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 50000);
      m_climberLead.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 50000);

      // Set the follow climber motor bandwidth to 200Hz (every 5ms) //
      m_climberFollow.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);

      // Set status 1-7 to be 50000ms for the follow climber motors
      m_climberFollow.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 50000);
      m_climberFollow.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50000);
      m_climberFollow.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 50000);
      m_climberFollow.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 50000);
      m_climberFollow.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 50000);
      m_climberFollow.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 50000);
      m_climberFollow.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 50000);
    }
  }

  /** Set the climber to a specific speed
   * @param speed the speed to set the climber to
   */
  public void set(double speed) {
    m_climberLead.set(speed);
  }

  /**
   * Set the climber to climb or descend
   * @param climb true to climb, false to descend
   */
  public void setClimber(boolean climb) {
    if (climb) {
      m_climberLead.set(1.0);
    } else {
      m_climberLead.set(-1.0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (GeneralConstants.CURRENT_MODE == RobotMode.TEST) {
      m_climberLeadPowerPercentageEntry.setDouble(m_climberLead.getAppliedOutput());
      m_climberLeadPowerVoltageEntry.setDouble(m_climberLead.getBusVoltage());
      m_climberLeadPowerCurrentEntry.setDouble(m_climberLead.getOutputCurrent());
      m_climberLeadPowerTemperatureEntry.setDouble(m_climberLead.getMotorTemperature());
      m_climberLeadPositionEntry.setDouble(m_climberLead.getEncoder().getPosition());

      m_climberFollowPowerPercentageEntry.setDouble(m_climberFollow.getAppliedOutput());
      m_climberFollowPowerVoltageEntry.setDouble(m_climberFollow.getBusVoltage());
      m_climberFollowPowerCurrentEntry.setDouble(m_climberFollow.getOutputCurrent());
      m_climberFollowPowerTemperatureEntry.setDouble(m_climberFollow.getMotorTemperature());
      m_climberFollowPositionEntry.setDouble(m_climberFollow.getEncoder().getPosition());
    }
  }
}
