// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AmpConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.GeneralConstants.RobotMode;

public class AmpSmartMotionSubsystem extends SubsystemBase {
  /** Creates a new AmpSmartMotionSubsystem. */

  // Create new Shuffleboard tab for the amp subsystem //
  private final ShuffleboardTab m_ampShuffleboardTab = Shuffleboard.getTab("Amp");
  private GenericEntry m_ampLeadPowerPercentageEntry = null;
  private GenericEntry m_ampLeadPowerVoltageEntry = null;
  private GenericEntry m_ampLeadPowerCurrentEntry = null;
  private GenericEntry m_ampLeadPowerTemperatureEntry = null;
  private GenericEntry m_ampLeadPositionEntry = null;
  private GenericEntry m_ampFollowPowerPercentageEntry = null;
  private GenericEntry m_ampFollowPowerVoltageEntry = null;
  private GenericEntry m_ampFollowPowerCurrentEntry = null;
  private GenericEntry m_ampFollowPowerTemperatureEntry = null;
  private GenericEntry m_ampFollowPositionEntry = null;

  // Setting Entries //
  public GenericEntry m_setAmpLeadPowerPercentageEntry = null;

  // Amp Motor Controllers //
  private final CANSparkMax m_ampLead = new CANSparkMax(AmpConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
  // private final CANSparkMax m_ampFollow = new CANSparkMax(AmpConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
  private final SparkPIDController m_ampLeadController = m_ampLead.getPIDController();
  // private final SparkPIDController m_ampFollowController = m_shooterFollow.getPIDController();
  private final SparkAbsoluteEncoder m_ampAbsEncoder = m_ampLead.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
  
  public AmpSmartMotionSubsystem() {

    // Restore motors to factory defaults //
    m_ampLead.restoreFactoryDefaults();
    // m_ampFollow.restoreFactoryDefaults();

    // Enable Voltage Compensation //
    m_ampLead.enableVoltageCompensation(AmpConstants.NOMINAL_VOLTAGE);
    // m_ampFollow.enableVoltageCompensation(AmpConstants.NOMINAL_VOLTAGE);

    // Set the smart current limits for the motors //
    m_ampLead.setSmartCurrentLimit(AmpConstants.STALL_CURRENT_LIMIT);
    // m_ampFollow.setSmartCurrentLimit(AmpConstants.STALL_CURRENT_LIMIT);

    // Set the secondary current limits for the motors //
    m_ampLead.setSecondaryCurrentLimit(AmpConstants.SECONDARY_CURRENT_LIMIT);
    // m_ampFollow.setSecondaryCurrentLimit(AmpConstants.SECONDARY_CURRENT_LIMIT);

    // Set the motor controllers to brake mode //
    m_ampLead.setIdleMode(CANSparkMax.IdleMode.kBrake);
    // m_ampFollow.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // Initialize the absolute encoder to 0//
    m_ampAbsEncoder.setZeroOffset(0.0);

    // PID Controller settings for the amp //
    m_ampLeadController.setP(AmpConstants.P);
    m_ampLeadController.setI(AmpConstants.I);
    m_ampLeadController.setD(AmpConstants.D);
    m_ampLeadController.setFF(AmpConstants.F);
    m_ampLeadController.setIZone(AmpConstants.IZ);
    m_ampLeadController.setOutputRange(AmpConstants.MIN_OUTPUT, AmpConstants.MAX_OUTPUT);

    m_ampLeadController.setSmartMotionMaxVelocity(AmpConstants.SMART_MOTION_MAX_VELOCITY, AmpConstants.SMART_MOTION_SLOT);
    m_ampLeadController.setSmartMotionMinOutputVelocity(AmpConstants.SMART_MOTION_MIN_OUTPUT_VELOCITY, AmpConstants.SMART_MOTION_SLOT);
    m_ampLeadController.setSmartMotionMaxAccel(AmpConstants.SMART_MOTION_MAX_ACCEL, AmpConstants.SMART_MOTION_SLOT);
    m_ampLeadController.setSmartMotionAllowedClosedLoopError(AmpConstants.SMART_MOTION_ALLOWED_ERROR, AmpConstants.SMART_MOTION_SLOT);

    // Set the lead amp to 0 RPM //
    m_ampLead.set(0);

    // Set the motor followers //
    // m_ampFollow.follow(m_ampLead, true);

    // Create Shuffleboard entries for the AmpSubsystem if the robot is in test mode //
    if (GeneralConstants.CURRENT_MODE == RobotMode.TEST) {
      m_ampLeadPowerPercentageEntry = m_ampShuffleboardTab.add("Amp Lead Power Percentage Reading", m_ampLead.getAppliedOutput()).getEntry();
      m_ampLeadPowerVoltageEntry = m_ampShuffleboardTab.add("Amp Lead Power Voltage Reading", m_ampLead.getBusVoltage()).getEntry();
      m_ampLeadPowerCurrentEntry = m_ampShuffleboardTab.add("Amp Lead Power Current Reading", m_ampLead.getOutputCurrent()).getEntry();
      m_ampLeadPowerTemperatureEntry = m_ampShuffleboardTab.add("Amp Lead Power Temperature Reading", m_ampLead.getMotorTemperature()).getEntry();
      m_ampLeadPositionEntry = m_ampShuffleboardTab.add("Amp Lead Position Reading", m_ampLead.getEncoder().getPosition()).getEntry();

      // m_ampFollowPowerPercentageEntry = m_ampShuffleboardTab.add("Amp Follow Power Percentage Reading", m_ampFollow.getAppliedOutput()).getEntry();
      // m_ampFollowPowerVoltageEntry = m_ampShuffleboardTab.add("Amp Follow Power Voltage Reading", m_ampFollow.getBusVoltage()).getEntry();
      // m_ampFollowPowerCurrentEntry = m_ampShuffleboardTab.add("Amp Follow Power Current Reading", m_ampFollow.getOutputCurrent()).getEntry();
      // m_ampFollowPowerTemperatureEntry = m_ampShuffleboardTab.add("Amp Follow Power Temperature Reading", m_ampFollow.getMotorTemperature()).getEntry();
      // m_ampFollowPositionEntry = m_ampShuffleboardTab.add("Amp Follow Position Reading", m_ampFollow.getEncoder().getPosition()).getEntry();

      m_setAmpLeadPowerPercentageEntry
        = m_ampShuffleboardTab.add("Amp Lead Power Percentage Setting", false)
          .withWidget(BuiltInWidgets.kToggleButton)
          .getEntry();
    }
  }

  //#region Amp Test Methods //
  
  public void setAmpSpeed(double speed) {
    m_ampLead.set(speed);
  }

  public void setAmpPosition(double position) {
    m_ampLeadController.setReference(position, ControlType.kSmartMotion);
  }

  //#endregion

  /**
   * Get the current position of the amp
   * @return
   */
  public double getAmpPosition() {
    return m_ampAbsEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (GeneralConstants.CURRENT_MODE == RobotMode.TEST) {
      m_ampLeadPowerPercentageEntry.setDouble(m_ampLead.getAppliedOutput());
      m_ampLeadPowerVoltageEntry.setDouble(m_ampLead.getBusVoltage());
      m_ampLeadPowerCurrentEntry.setDouble(m_ampLead.getOutputCurrent());
      m_ampLeadPowerTemperatureEntry.setDouble(m_ampLead.getMotorTemperature());
      m_ampLeadPositionEntry.setDouble(m_ampLead.getEncoder().getPosition());

      // m_ampFollowPowerPercentageEntry.setDouble(m_ampFollow.getAppliedOutput());
      // m_ampFollowPowerVoltageEntry.setDouble(m_ampFollow.getBusVoltage());
      // m_ampFollowPowerCurrentEntry.setDouble(m_ampFollow.getOutputCurrent());
      // m_ampFollowPowerTemperatureEntry.setDouble(m_ampFollow.getMotorTemperature());
      // m_ampFollowPositionEntry.setDouble(m_ampFollow.getEncoder().getPosition());
    }
  }
}
