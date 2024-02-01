// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  
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

    /* Follow motor settings */    
    // Restore follow motor to factory default //
    m_climberFollow.restoreFactoryDefaults();

    // Set the follow motor to only use 10 volts of power //
    m_climberFollow.enableVoltageCompensation(ClimberConstants.NOMINAL_VOLTAGE);

    // Set the follow motor to only use 40 amp stall limit //
    m_climberFollow.setSmartCurrentLimit(ClimberConstants.STALL_CURRENT_LIMIT);

    // Set the follow motor's hard stop current limit to 60 amps //
    m_climberFollow.setSecondaryCurrentLimit(ClimberConstants.SECONDARY_CURRENT_LIMIT);
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
  }
}
