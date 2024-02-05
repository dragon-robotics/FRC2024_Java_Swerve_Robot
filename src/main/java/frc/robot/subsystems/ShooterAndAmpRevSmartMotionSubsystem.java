// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AmpConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterAndAmpRevSmartMotionSubsystem extends SubsystemBase {

  // Shooter motor controllers //
  private static final CANSparkMax m_shooterLead = new CANSparkMax(ShooterConstants.TOP_MOTOR_ID, MotorType.kBrushless);
  private static final CANSparkMax m_shooterFollow = new CANSparkMax(ShooterConstants.BOTTOM_MOTOR_ID, MotorType.kBrushless);
  private static final SparkPIDController m_shooterLeadController = m_shooterLead.getPIDController();
  // private static final SparkPIDController m_shooterFollowController = m_shooterFollow.getPIDController();

  // Amp motor controllers //
  private static final CANSparkMax m_ampLead = new CANSparkMax(AmpConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
  private static final CANSparkMax m_ampFollow = new CANSparkMax(AmpConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
  private static final SparkPIDController m_ampLeadController = m_shooterLead.getPIDController();
  // private static final SparkPIDController m_ampFollowController = m_shooterFollow.getPIDController();


  /** Creates a new ShooterAndAmpRevSmartMotionSubsystem. */
  public ShooterAndAmpRevSmartMotionSubsystem() {

    // Restore motors to factory defaults //
    m_shooterLead.restoreFactoryDefaults();
    m_shooterFollow.restoreFactoryDefaults();
    m_ampLead.restoreFactoryDefaults();
    m_ampFollow.restoreFactoryDefaults();

    // Enable Voltage Compensation //
    m_shooterLead.enableVoltageCompensation(ShooterConstants.NOMINAL_VOLTAGE);
    m_shooterFollow.enableVoltageCompensation(ShooterConstants.NOMINAL_VOLTAGE);
    m_ampLead.enableVoltageCompensation(AmpConstants.NOMINAL_VOLTAGE);
    m_ampFollow.enableVoltageCompensation(AmpConstants.NOMINAL_VOLTAGE);

    // Set the smart current limits for the motors //
    m_shooterLead.setSmartCurrentLimit(ShooterConstants.STALL_CURRENT_LIMIT, ShooterConstants.FREE_CURRENT_LIMIT);
    m_shooterFollow.setSmartCurrentLimit(ShooterConstants.STALL_CURRENT_LIMIT, ShooterConstants.FREE_CURRENT_LIMIT);
    m_ampLead.setSmartCurrentLimit(AmpConstants.STALL_CURRENT_LIMIT);
    m_ampFollow.setSmartCurrentLimit(AmpConstants.STALL_CURRENT_LIMIT);

    // Set the secondary current limits for the motors //
    m_shooterLead.setSecondaryCurrentLimit(ShooterConstants.SECONDARY_CURRENT_LIMIT);
    m_shooterFollow.setSecondaryCurrentLimit(ShooterConstants.SECONDARY_CURRENT_LIMIT);
    m_ampLead.setSecondaryCurrentLimit(AmpConstants.SECONDARY_CURRENT_LIMIT);
    m_ampFollow.setSecondaryCurrentLimit(AmpConstants.SECONDARY_CURRENT_LIMIT);

    // Set the motor controllers to brake mode //
    m_shooterLead.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_shooterFollow.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_ampLead.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_ampFollow.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // Set the follower motor to follow the lead motor //
    m_shooterFollow.follow(m_shooterLead);
    m_ampFollow.follow(m_ampLead);

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

    m_shooterLeadController.setReference(0, ControlType.kSmartVelocity);

    // Set the lead shooter to 0 RPM //
    m_shooterLead.set(0);


    // PID Controller settings for the amp
    m_ampLeadController.setP(AmpConstants.P);
    m_ampLeadController.setI(AmpConstants.I);
    m_ampLeadController.setD(AmpConstants.D);
    m_ampLeadController.setFF(AmpConstants.F);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
