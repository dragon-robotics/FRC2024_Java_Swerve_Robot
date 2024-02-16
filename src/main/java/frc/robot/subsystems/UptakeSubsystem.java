// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.UptakeConstants;
import frc.robot.Constants.GeneralConstants.RobotMode;

public class UptakeSubsystem extends SubsystemBase {
  /** Creates a new UptakeSubsystem. */
  
// Create new Shuffleboard tab for the uptake subsystem //
  private final ShuffleboardTab m_uptakeShuffleboardTab = Shuffleboard.getTab("Uptake");
  private GenericEntry m_uptakeLeadPowerPercentageEntry = null;
  private GenericEntry m_uptakeLeadPowerVoltageEntry = null;
  private GenericEntry m_uptakeLeadPowerCurrentEntry = null;
  private GenericEntry m_uptakeLeadPowerTemperatureEntry = null;
  private GenericEntry m_uptakeLeadPowerRPMEntry = null;
  private GenericEntry m_uptakeFollowPowerPercentageEntry = null;
  private GenericEntry m_uptakeFollowPowerVoltageEntry = null;
  private GenericEntry m_uptakeFollowPowerCurrentEntry = null;
  private GenericEntry m_uptakeFollowPowerTemperatureEntry = null;
  private GenericEntry m_uptakeFollowPowerRPMEntry = null;

  public GenericEntry m_uptakeLeadPowerPercentageSetEntry = null;

  private final CANSparkMax m_uptakeLead = new CANSparkMax(UptakeConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax m_uptakeFollow = new CANSparkMax(UptakeConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

  // private final DigitalInput m_topBeamBreak = new DigitalInput(UptakeConstants.TOP_BEAM_BREAK_DIGITAL_CHANNEL);
  // private final DigitalInput m_bottomBeamBreak = new DigitalInput(UptakeConstants.BOTTOM_BEAM_BREAK_DIGITAL_CHANNEL);

  /**
   * Creates a new UptakeSubsystem.
   */
  public UptakeSubsystem() {

    // #region Lead motor settings

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

    // #endregion

    // #region Follow motor settings

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
    m_uptakeFollow.follow(m_uptakeLead, true);

    // #endregion

    // Set the motors initially to 0 speed //
    m_uptakeLead.set(0.0);

    // Create Shuffleboard entries for the UptakeSubsystem if the robot is in test mode //
    if (GeneralConstants.CURRENT_MODE == RobotMode.TEST) {
      m_uptakeLeadPowerPercentageEntry = m_uptakeShuffleboardTab.add("Uptake Lead Power Percentage Reading", m_uptakeLead.getAppliedOutput()).getEntry();
      m_uptakeLeadPowerVoltageEntry = m_uptakeShuffleboardTab.add("Uptake Lead Power Voltage Reading", m_uptakeLead.getBusVoltage()).getEntry();
      m_uptakeLeadPowerCurrentEntry = m_uptakeShuffleboardTab.add("Uptake Lead Power Current Reading", m_uptakeLead.getOutputCurrent()).getEntry();
      m_uptakeLeadPowerTemperatureEntry = m_uptakeShuffleboardTab.add("Uptake Lead Power Temperature Reading", m_uptakeLead.getMotorTemperature()).getEntry();
      m_uptakeLeadPowerRPMEntry = m_uptakeShuffleboardTab.add("Uptake Lead Power RPM Reading", m_uptakeLead.getEncoder().getVelocity()).getEntry();

      m_uptakeFollowPowerPercentageEntry = m_uptakeShuffleboardTab.add("Uptake Follow Power Percentage Reading", m_uptakeFollow.getAppliedOutput()).getEntry();
      m_uptakeFollowPowerVoltageEntry = m_uptakeShuffleboardTab.add("Uptake Follow Power Voltage Reading", m_uptakeFollow.getBusVoltage()).getEntry();
      m_uptakeFollowPowerCurrentEntry = m_uptakeShuffleboardTab.add("Uptake Follow Power Current Reading", m_uptakeFollow.getOutputCurrent()).getEntry();
      m_uptakeFollowPowerTemperatureEntry = m_uptakeShuffleboardTab.add("Uptake Follow Power Temperature Reading", m_uptakeFollow.getMotorTemperature()).getEntry();
      m_uptakeFollowPowerRPMEntry = m_uptakeShuffleboardTab.add("Uptake Follow Power RPM Reading", m_uptakeFollow.getEncoder().getVelocity()).getEntry();

      m_uptakeLeadPowerPercentageSetEntry
        = m_uptakeShuffleboardTab.add("Uptake Lead Power Percentage Setting", 0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .getEntry();
    }
  }

  public void setSpeedForward100() {
    m_uptakeLead.set(1);
  }

  public void setSpeedReverse100() {
    m_uptakeLead.set(-1);
  }

  public void setSpeed0() {
    m_uptakeLead.set(0);
  }

  // /**
  //  * Get the state of the top beam break sensor
  //  * @return true if the top beam break sensor is broken, false if it is not
  //  */
  // public boolean getTopBeamBreak() {
  //   return m_topBeamBreak.get();
  // }

  // /**
  //  * Get the state of the bottom beam break sensor
  //  * @return true if the bottom beam break sensor is broken, false if it is not
  //  */
  // public boolean getBottomBeamBreak() {
  //   return m_bottomBeamBreak.get();
  // }

  /**
   * Set the intake to uptake or downtake
   * @param uptake true to uptake, false to downtake
   */
  public void setUptake(boolean uptake) {
    if (uptake) {
      m_uptakeLead.set(1.0);
    } else {
      m_uptakeLead.set(-1.0);
    }
  } 

  /**
   * Set the uptake to a specific speed
   * @param speed the speed to set the uptake to
   */
  public void set(double speed) {
    m_uptakeLead.set(speed);
  }

  /**
   * Set the uptake to uptake at 100% speed
   */
  public void uptake100() {
    m_uptakeLead.set(1);
  }

  /**
   * Set the uptake to uptake at 40% speed
   */
  public void uptake40() {
    m_uptakeLead.set(0.4);
  }

  /**
   * Set the uptake to downtake at 100% speed
   */
  public void downtake100() {
    m_uptakeLead.set(-1);
  }

  /**
   * Set the uptake to downtake at 40% speed
   */
  public void downtake40() {
    m_uptakeLead.set(-0.4);
  }

  /**
   * Stop the uptake
   */
  public void stopUptake() {
    m_uptakeLead.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (GeneralConstants.CURRENT_MODE == RobotMode.TEST) {
      m_uptakeLeadPowerPercentageEntry.setDouble(m_uptakeLead.getAppliedOutput());
      m_uptakeLeadPowerVoltageEntry.setDouble(m_uptakeLead.getBusVoltage());
      m_uptakeLeadPowerCurrentEntry.setDouble(m_uptakeLead.getOutputCurrent());
      m_uptakeLeadPowerTemperatureEntry.setDouble(m_uptakeLead.getMotorTemperature());
      m_uptakeLeadPowerRPMEntry.setDouble(m_uptakeLead.getEncoder().getVelocity());

      m_uptakeFollowPowerPercentageEntry.setDouble(m_uptakeFollow.getAppliedOutput());
      m_uptakeFollowPowerVoltageEntry.setDouble(m_uptakeFollow.getBusVoltage());
      m_uptakeFollowPowerCurrentEntry.setDouble(m_uptakeFollow.getOutputCurrent());
      m_uptakeFollowPowerTemperatureEntry.setDouble(m_uptakeFollow.getMotorTemperature());
      m_uptakeFollowPowerRPMEntry.setDouble(m_uptakeFollow.getEncoder().getVelocity());
    }
  }
}
