// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AmpConstants;
import frc.robot.Constants.GeneralConstants.RobotMode;

public class AmpSmartMotionSubsystem extends SubsystemBase {
  /** Creates a new AmpSmartMotionSubsystem. */

  // Create New Tab for Shooter and Amp subsystems //
  private final ShuffleboardTab m_ampShuffleboardTab = Shuffleboard.getTab("Amp");

  // Amp Motor Controllers //
  private final CANSparkMax m_ampLead = new CANSparkMax(AmpConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax m_ampFollow = new CANSparkMax(AmpConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
  private final SparkPIDController m_ampLeadController = m_ampLead.getPIDController();
  // private final SparkPIDController m_ampFollowController = m_shooterFollow.getPIDController();
  
  public AmpSmartMotionSubsystem(RobotMode mode) {

    // Restore motors to factory defaults //
    m_ampLead.restoreFactoryDefaults();
    m_ampFollow.restoreFactoryDefaults();

    // Enable Voltage Compensation //
    m_ampLead.enableVoltageCompensation(AmpConstants.NOMINAL_VOLTAGE);
    m_ampFollow.enableVoltageCompensation(AmpConstants.NOMINAL_VOLTAGE);

    // Set the smart current limits for the motors //
    m_ampLead.setSmartCurrentLimit(AmpConstants.STALL_CURRENT_LIMIT);
    m_ampFollow.setSmartCurrentLimit(AmpConstants.STALL_CURRENT_LIMIT);

    // Set the secondary current limits for the motors //
    m_ampLead.setSecondaryCurrentLimit(AmpConstants.SECONDARY_CURRENT_LIMIT);
    m_ampFollow.setSecondaryCurrentLimit(AmpConstants.SECONDARY_CURRENT_LIMIT);

    // Set the motor controllers to brake mode //
    m_ampLead.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_ampFollow.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // PID Controller settings for the amp //
    m_ampLeadController.setP(AmpConstants.P);
    m_ampLeadController.setI(AmpConstants.I);
    m_ampLeadController.setD(AmpConstants.D);
    m_ampLeadController.setFF(AmpConstants.F);
    m_ampLeadController.setIZone(AmpConstants.IZ);

    // Set the lead amp to 0 RPM //
    m_ampLead.set(0);

    // Set the motor followers //
    m_ampFollow.follow(m_ampLead, true);
  }

  //#region Amp Test Methods //
  
  public void setAmpSpeed(double speed) {
    m_ampLead.set(speed * 0.3);
  }

  public void setAmpPosition(double position) {
    m_ampLeadController.setReference(position, ControlType.kSmartMotion);
  }

  //#endregion

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
