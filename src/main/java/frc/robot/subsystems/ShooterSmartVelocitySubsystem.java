// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.GeneralConstants.RobotMode;

public class ShooterSmartVelocitySubsystem extends SubsystemBase {

  // Create New Tab for Shooter and Amp subsystems //
  private final ShuffleboardTab m_shooterShuffleboardTab = Shuffleboard.getTab("Shooter");

  // Shooter Motor Controllers //
  private final CANSparkMax m_shooterLead = new CANSparkMax(ShooterConstants.TOP_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax m_shooterFollow = new CANSparkMax(ShooterConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
  private final SparkPIDController m_shooterLeadController = m_shooterLead.getPIDController();
  // private final SparkPIDController m_shooterFollowController = m_shooterFollow.getPIDController();

  // Shooter Beambreak Sensor //
  private final DigitalInput m_shooterBeamBreak = new DigitalInput(ShooterConstants.BEAM_BREAK_DIGITAL_CHANNEL);

  /** Creates a new ShooterAndAmpRevSmartMotionSubsystem. */
  public ShooterSmartVelocitySubsystem(RobotMode mode) {

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
  }

  //#region Shooter Test Methods //

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
