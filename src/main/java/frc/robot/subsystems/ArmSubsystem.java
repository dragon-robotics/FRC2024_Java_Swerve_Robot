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
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.GeneralConstants.RobotMode;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSmartMotionSubsystem. */

  // Create new Shuffleboard tab for the arm subsystem //
  private final ShuffleboardTab m_armShuffleboardTab = Shuffleboard.getTab("Arm");
  private GenericEntry m_armLeadPowerPercentageEntry = null;
  private GenericEntry m_armLeadPowerVoltageEntry = null;
  private GenericEntry m_armLeadPowerCurrentEntry = null;
  private GenericEntry m_armLeadPowerTemperatureEntry = null;
  private GenericEntry m_armLeadPositionEntry = null;
  private GenericEntry m_armFollowPowerPercentageEntry = null;
  private GenericEntry m_armFollowPowerVoltageEntry = null;
  private GenericEntry m_armFollowPowerCurrentEntry = null;
  private GenericEntry m_armFollowPowerTemperatureEntry = null;
  private GenericEntry m_armFollowPositionEntry = null;

  // Setting Entries //
  public GenericEntry m_setArmLeadPowerPercentageEntry = null;

  // Arm Motor Controllers //
  private final CANSparkMax m_armLead = new CANSparkMax(ArmConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
  // private final CANSparkMax m_armFollow = new CANSparkMax(ArmConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
  private final SparkPIDController m_armLeadController = m_armLead.getPIDController();
  // private final SparkPIDController m_armFollowController = m_shooterFollow.getPIDController();
  private final SparkAbsoluteEncoder m_armAbsEncoder = m_armLead.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
  
  public ArmSubsystem() {

    // Restore motors to factory defaults //
    m_armLead.restoreFactoryDefaults();
    // m_armFollow.restoreFactoryDefaults();

    // Enable Voltage Compensation //
    m_armLead.enableVoltageCompensation(ArmConstants.NOMINAL_VOLTAGE);
    // m_armFollow.enableVoltageCompensation(ArmConstants.NOMINAL_VOLTAGE);

    // Set the smart current limits for the motors //
    m_armLead.setSmartCurrentLimit(ArmConstants.STALL_CURRENT_LIMIT);
    // m_armFollow.setSmartCurrentLimit(ArmConstants.STALL_CURRENT_LIMIT);

    // Set the secondary current limits for the motors //
    m_armLead.setSecondaryCurrentLimit(ArmConstants.SECONDARY_CURRENT_LIMIT);
    // m_armFollow.setSecondaryCurrentLimit(ArmConstants.SECONDARY_CURRENT_LIMIT);

    // Set the motor controllers to brake mode //
    m_armLead.setIdleMode(CANSparkMax.IdleMode.kBrake);
    // m_armFollow.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // Initialize the absolute encoder to 0//
    m_armAbsEncoder.setZeroOffset(0.0);

    // PID Controller settings for the arm //
    m_armLeadController.setP(ArmConstants.P);
    m_armLeadController.setI(ArmConstants.I);
    m_armLeadController.setD(ArmConstants.D);
    m_armLeadController.setFF(ArmConstants.F);
    m_armLeadController.setIZone(ArmConstants.IZ);
    m_armLeadController.setOutputRange(ArmConstants.MIN_OUTPUT, ArmConstants.MAX_OUTPUT);

    m_armLeadController.setSmartMotionMaxVelocity(ArmConstants.SMART_MOTION_MAX_VELOCITY, ArmConstants.SMART_MOTION_SLOT);
    m_armLeadController.setSmartMotionMinOutputVelocity(ArmConstants.SMART_MOTION_MIN_OUTPUT_VELOCITY, ArmConstants.SMART_MOTION_SLOT);
    m_armLeadController.setSmartMotionMaxAccel(ArmConstants.SMART_MOTION_MAX_ACCEL, ArmConstants.SMART_MOTION_SLOT);
    m_armLeadController.setSmartMotionAllowedClosedLoopError(ArmConstants.SMART_MOTION_ALLOWED_ERROR, ArmConstants.SMART_MOTION_SLOT);

    // Set the lead arm to 0 RPM //
    m_armLead.set(0);

    // Set the motor followers //
    // m_armFollow.follow(m_armLead, true);

    // Create Shuffleboard entries for the ArmSubsystem if the robot is in test mode //
    if (GeneralConstants.CURRENT_MODE == RobotMode.TEST) {
      m_armLeadPowerPercentageEntry = m_armShuffleboardTab.add("Arm Lead Power Percentage Reading", m_armLead.getAppliedOutput()).getEntry();
      m_armLeadPowerVoltageEntry = m_armShuffleboardTab.add("Arm Lead Power Voltage Reading", m_armLead.getBusVoltage()).getEntry();
      m_armLeadPowerCurrentEntry = m_armShuffleboardTab.add("Arm Lead Power Current Reading", m_armLead.getOutputCurrent()).getEntry();
      m_armLeadPowerTemperatureEntry = m_armShuffleboardTab.add("Arm Lead Power Temperature Reading", m_armLead.getMotorTemperature()).getEntry();
      m_armLeadPositionEntry = m_armShuffleboardTab.add("Arm Lead Position Reading", m_armLead.getEncoder().getPosition()).getEntry();

      // m_armFollowPowerPercentageEntry = m_armShuffleboardTab.add("Arm Follow Power Percentage Reading", m_armFollow.getAppliedOutput()).getEntry();
      // m_armFollowPowerVoltageEntry = m_armShuffleboardTab.add("Arm Follow Power Voltage Reading", m_armFollow.getBusVoltage()).getEntry();
      // m_armFollowPowerCurrentEntry = m_armShuffleboardTab.add("Arm Follow Power Current Reading", m_armFollow.getOutputCurrent()).getEntry();
      // m_armFollowPowerTemperatureEntry = m_armShuffleboardTab.add("Arm Follow Power Temperature Reading", m_armFollow.getMotorTemperature()).getEntry();
      // m_armFollowPositionEntry = m_armShuffleboardTab.add("Arm Follow Position Reading", m_armFollow.getEncoder().getPosition()).getEntry();

      m_setArmLeadPowerPercentageEntry
        = m_armShuffleboardTab.add("Arm Lead Power Percentage Setting", false)
          .withWidget(BuiltInWidgets.kToggleButton)
          .getEntry();
    }
  }

  //#region Arm Test Methods //
  
  public void setArmSpeed(double speed) {
    m_armLead.set(speed);
  }

  public void setArmPosition(double position) {
    m_armLeadController.setReference(position, ControlType.kSmartMotion);
  }

  public void stopArm() {
    m_armLead.set(0);
  }

  //#endregion

  /**
   * Get the current position of the amp
   * @return
   */
  public double getArmPosition() {
    return m_armAbsEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (GeneralConstants.CURRENT_MODE == RobotMode.TEST) {
      m_armLeadPowerPercentageEntry.setDouble(m_armLead.getAppliedOutput());
      m_armLeadPowerVoltageEntry.setDouble(m_armLead.getBusVoltage());
      m_armLeadPowerCurrentEntry.setDouble(m_armLead.getOutputCurrent());
      m_armLeadPowerTemperatureEntry.setDouble(m_armLead.getMotorTemperature());
      m_armLeadPositionEntry.setDouble(m_armLead.getEncoder().getPosition());

      // m_armFollowPowerPercentageEntry.setDouble(m_armFollow.getAppliedOutput());
      // m_armFollowPowerVoltageEntry.setDouble(m_armFollow.getBusVoltage());
      // m_armFollowPowerCurrentEntry.setDouble(m_armFollow.getOutputCurrent());
      // m_armFollowPowerTemperatureEntry.setDouble(m_armFollow.getMotorTemperature());
      // m_armFollowPositionEntry.setDouble(m_armFollow.getEncoder().getPosition());
    }
  }
}
