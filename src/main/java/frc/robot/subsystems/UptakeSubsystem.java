// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.UptakeConstants;

public class UptakeSubsystem extends SubsystemBase {
  /** Creates a new UptakeSubsystem. */
  
  private final CANSparkMax m_uptakeLead = new CANSparkMax(UptakeConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax m_uptakeFollow = new CANSparkMax(UptakeConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

  /**
   * Creates a new UptakeSubsystem.
   */
  public UptakeSubsystem() {

    /* Lead motor settings */

    // Restore lead motor to factory default //
    m_uptakeLead.restoreFactoryDefaults();

    // Set the lead motor to only use 8 volts of power //
    m_uptakeLead.enableVoltageCompensation(UptakeConstants.NOMINAL_VOLTAGE);

    // Set the lead motor to only use 20 amp stall limit, 10 amp free limit //
    m_uptakeLead.setSmartCurrentLimit(
        UptakeConstants.STALL_CURRENT_LIMIT,
        UptakeConstants.FREE_CURRENT_LIMIT
    );

    // Set the lead motor's hard stop current limit to 40 amps //
    m_uptakeLead.setSecondaryCurrentLimit(UptakeConstants.SECONDARY_CURRENT_LIMIT);

    // Set the lead motor to brake mode //
    m_uptakeLead.setIdleMode(IdleMode.kBrake);

    // Set lead motor ramp rate to 0.25 seconds //
    m_uptakeLead.setOpenLoopRampRate(UptakeConstants.RAMP_RATE_IN_SEC);

    /* Follower motor settings */
    // Restore follow motor to factory default //
    m_uptakeFollow.restoreFactoryDefaults();

    // Set the follow motor to only use 8 volts of power //
    m_uptakeFollow.enableVoltageCompensation(UptakeConstants.NOMINAL_VOLTAGE);

    // Set the follow motor to only use 20 amp stall limit, 10 amp free limit //
    m_uptakeFollow.setSmartCurrentLimit(
        UptakeConstants.STALL_CURRENT_LIMIT,
        UptakeConstants.FREE_CURRENT_LIMIT
    );

    // Set the follow motor's hard stop current limit to 40 amps //
    m_uptakeFollow.setSecondaryCurrentLimit(UptakeConstants.SECONDARY_CURRENT_LIMIT);

    // Set the follow motor to brake mode //
    m_uptakeFollow.setIdleMode(IdleMode.kBrake);

    // Set follow motor ramp rate to 0.25 seconds //
    m_uptakeFollow.setOpenLoopRampRate(UptakeConstants.RAMP_RATE_IN_SEC);

    // Make the follow motor follow the lead motor //
    m_uptakeFollow.follow(m_uptakeLead);
  }

  /**
   * Set the intake to intake or outtake
   * @param intake true to intake, false to outtake
   */
  public void setIntake(boolean uptake) {
    if (uptake) {
      m_uptakeLead.set(1.0);
    } else {
      m_uptakeLead.set(-1.0);
    }
  } 

  /**
   * Set the intake to a specific speed
   * @param speed the speed to set the intake to
   */
  public void set(double speed) {
    m_uptakeLead.set(speed);
  }

  public void uptake100() {
    m_uptakeLead.set(1);
  }

  public void uptake40() {
    m_uptakeLead.set(0.4);
  }

  public void downtake100() {
    m_uptakeLead.set(-1);
  }

  public void downtake40() {
    m_uptakeLead.set(-0.4);
  }

  /**
   * Stop the uptake
   */
  public void stopUptake() {
    m_uptakeLead.set(0.0);
  }
}
