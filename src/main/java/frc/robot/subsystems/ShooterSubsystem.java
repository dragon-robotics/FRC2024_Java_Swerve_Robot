// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.GeneralConstants.RobotMode;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  // Create new Shuffleboard tab for shooter subsystem //
  private final ShuffleboardTab m_shooterShuffleboardTab = Shuffleboard.getTab("Shooter");
  private GenericEntry m_shooterLeadPowerPercentageEntry = null;
  private GenericEntry m_shooterLeadPowerVoltageEntry = null;
  private GenericEntry m_shooterLeadPowerCurrentEntry = null;
  private GenericEntry m_shooterLeadPowerTemperatureEntry = null;
  private GenericEntry m_shooterLeadPowerRPMEntry = null;
  private GenericEntry m_shooterFollowPowerPercentageEntry = null;
  private GenericEntry m_shooterFollowPowerVoltageEntry = null;
  private GenericEntry m_shooterFollowPowerCurrentEntry = null;
  private GenericEntry m_shooterFollowPowerTemperatureEntry = null;
  private GenericEntry m_shooterFollowPowerRPMEntry = null;

  public GenericEntry m_shooterLeadPowerPercentageSetEntry = null;

  // Shooter Motor Controllers //
  private final CANSparkMax m_shooterLead = new CANSparkMax(ShooterConstants.TOP_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax m_shooterFollow = new CANSparkMax(ShooterConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
  private final SparkPIDController m_shooterLeadController = m_shooterLead.getPIDController();
  // private final SparkPIDController m_shooterFollowController = m_shooterFollow.getPIDController();

  // // Shooter Note Beambreak Sensor //
  // private final DigitalInput m_noteBeamBreak = new DigitalInput(ShooterConstants.NOTE_BEAM_BREAK_DIGITAL_CHANNEL);

  /** Creates a new ShooterSmartVelocitySubsystem. */
  public ShooterSubsystem() {

    // Restore motors to factory defaults //
    m_shooterLead.restoreFactoryDefaults();
    m_shooterFollow.restoreFactoryDefaults();

    // Enable Voltage Compensation //
    m_shooterLead.enableVoltageCompensation(ShooterConstants.NOMINAL_VOLTAGE);
    m_shooterFollow.enableVoltageCompensation(ShooterConstants.NOMINAL_VOLTAGE);

    // Set the smart current limits for the motors //
    m_shooterLead.setSmartCurrentLimit(ShooterConstants.STALL_CURRENT_LIMIT, ShooterConstants.FREE_CURRENT_LIMIT);
    m_shooterFollow.setSmartCurrentLimit(ShooterConstants.STALL_CURRENT_LIMIT, ShooterConstants.FREE_CURRENT_LIMIT);

    // Set the secondary current limits for the motors //
    m_shooterLead.setSecondaryCurrentLimit(ShooterConstants.SECONDARY_CURRENT_LIMIT);
    m_shooterFollow.setSecondaryCurrentLimit(ShooterConstants.SECONDARY_CURRENT_LIMIT);

    // Set the motor controllers to brake mode //
    m_shooterLead.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_shooterFollow.setIdleMode(CANSparkMax.IdleMode.kCoast);

    // Set the follower motor to follow the lead motor //
    m_shooterFollow.follow(m_shooterLead);

    // Set the motor controllers to ramp at a certain rate //
    m_shooterLead.setOpenLoopRampRate(ShooterConstants.RAMP_RATE_IN_SEC);
    m_shooterFollow.setOpenLoopRampRate(ShooterConstants.RAMP_RATE_IN_SEC);

    // PID Controller settings for the shooter //
    m_shooterLeadController.setP(ShooterConstants.P);
    m_shooterLeadController.setI(ShooterConstants.I);
    m_shooterLeadController.setD(ShooterConstants.D);
    m_shooterLeadController.setFF(ShooterConstants.F);
    m_shooterLeadController.setIZone(ShooterConstants.IZ);
    m_shooterLeadController.setOutputRange(ShooterConstants.MIN_OUTPUT, ShooterConstants.MAX_OUTPUT);

    // Set the lead shooter to 0 RPM //
    m_shooterLead.set(0);

    // Create Shuffleboard entries for the ShooterSubsystem if the robot is in test mode //
    if (GeneralConstants.CURRENT_MODE == RobotMode.TEST) {
      m_shooterLeadPowerPercentageEntry = m_shooterShuffleboardTab.add("Shooter Lead Power Percentage Reading", m_shooterLead.getAppliedOutput()).getEntry();
      m_shooterLeadPowerVoltageEntry = m_shooterShuffleboardTab.add("Shooter Lead Power Voltage Reading", m_shooterLead.getBusVoltage()).getEntry();
      m_shooterLeadPowerCurrentEntry = m_shooterShuffleboardTab.add("Shooter Lead Power Current Reading", m_shooterLead.getOutputCurrent()).getEntry();
      m_shooterLeadPowerTemperatureEntry = m_shooterShuffleboardTab.add("Shooter Lead Power Temperature Reading", m_shooterLead.getMotorTemperature()).getEntry();
      m_shooterLeadPowerRPMEntry = m_shooterShuffleboardTab.add("Shooter Lead Power RPM Reading", m_shooterLead.getEncoder().getVelocity()).getEntry();

      m_shooterFollowPowerPercentageEntry = m_shooterShuffleboardTab.add("Shooter Follow Power Percentage Reading", m_shooterFollow.getAppliedOutput()).getEntry();
      m_shooterFollowPowerVoltageEntry = m_shooterShuffleboardTab.add("Shooter Follow Power Voltage Reading", m_shooterFollow.getBusVoltage()).getEntry();
      m_shooterFollowPowerCurrentEntry = m_shooterShuffleboardTab.add("Shooter Follow Power Current Reading", m_shooterFollow.getOutputCurrent()).getEntry();
      m_shooterFollowPowerTemperatureEntry = m_shooterShuffleboardTab.add("Shooter Follow Power Temperature Reading", m_shooterFollow.getMotorTemperature()).getEntry();
      m_shooterFollowPowerRPMEntry = m_shooterShuffleboardTab.add("Shooter Follow Power RPM Reading", m_shooterFollow.getEncoder().getVelocity()).getEntry();

      m_shooterLeadPowerPercentageSetEntry
        = m_shooterShuffleboardTab.add("Shooter Lead Power Percentage Setting", 0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .getEntry();
    }
  }

  //#region Shooter Test Methods //

  public void set(double speed) {
    m_shooterLead.set(speed);
  }

  /**
   * Set the shooter RPM
   * @param rpm
   */
  public void setShooterRPM(double rpm) {
    m_shooterLeadController.setReference(rpm, ControlType.kVelocity);
  }

  /**
   * Get the shooter RPM
   * @return
   */
  public double getShooterRPM() {
    return m_shooterLead.getEncoder().getVelocity();
  }

  /**
   * Set the shooter percent output
   * @param percentOutput
   */
  public void setShooterPercentOutput(double percentOutput) {
    m_shooterLead.set(percentOutput);
  }

  /**
   * Set the shooter P variable
   * @return
   */
  public void setShooterP(double p){
    m_shooterLeadController.setP(p);
  }

  /**
   * Set the shooter I variable
   * @return
   */
  public void setShooterI(double i){
    m_shooterLeadController.setI(i);
  }

  /**
   * Set the shooter D variable
   * @return
   */
  public void setShooterD(double d){
    m_shooterLeadController.setD(d);
  }

  /**
   * Set the shooter F variable
   * @return
   */
  public void setShooterF(double f){
    m_shooterLeadController.setFF(f);
  }

  //#endregion //

  // /**
  //  * Determine if a note is detected by the shooter note beam break sensor
  //  * @return true if a note is detected, false if not
  //  */
  // public boolean isNoteDetected() {
  //   return m_noteBeamBreak.get();
  // }

  /**
   * Stop the shooter
   */
  public void stopShooter() {
    m_shooterLead.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (GeneralConstants.CURRENT_MODE == RobotMode.TEST) {
      m_shooterLeadPowerPercentageEntry.setDouble(m_shooterLead.getAppliedOutput());
      m_shooterLeadPowerVoltageEntry.setDouble(m_shooterLead.getBusVoltage());
      m_shooterLeadPowerCurrentEntry.setDouble(m_shooterLead.getOutputCurrent());
      m_shooterLeadPowerTemperatureEntry.setDouble(m_shooterLead.getMotorTemperature());
      m_shooterLeadPowerRPMEntry.setDouble(m_shooterLead.getEncoder().getVelocity());

      m_shooterFollowPowerPercentageEntry.setDouble(m_shooterFollow.getAppliedOutput());
      m_shooterFollowPowerVoltageEntry.setDouble(m_shooterFollow.getBusVoltage());
      m_shooterFollowPowerCurrentEntry.setDouble(m_shooterFollow.getOutputCurrent());
      m_shooterFollowPowerTemperatureEntry.setDouble(m_shooterFollow.getMotorTemperature());
      m_shooterFollowPowerRPMEntry.setDouble(m_shooterFollow.getEncoder().getVelocity());
    }
  }
}
