// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  
  private final CANSparkMax m_intake = new CANSparkMax(IntakeConstants.MOTOR_ID, MotorType.kBrushless);

  /**
   * Creates a new IntakeSubsystem.
   */
  public IntakeSubsystem() {
    // Restore motor to factory default //
    m_intake.restoreFactoryDefaults();

    // Set the motor to only use 8 volts of power //
    m_intake.enableVoltageCompensation(IntakeConstants.NOMINAL_VOLTAGE);

    // Set the motor to only use 20 amp stall limit, 10 free limit, and limit to 3k RPM //
    m_intake.setSmartCurrentLimit(
        IntakeConstants.STALL_CURRENT_LIMIT,
        IntakeConstants.FREE_CURRENT_LIMIT,
        IntakeConstants.RPM_LIMIT
    );

    // Set the motor's hard stop current limit to 40 amps //
    m_intake.setSecondaryCurrentLimit(IntakeConstants.SECONDARY_CURRENT_LIMIT);

    // Set the motor to brake mode //
    m_intake.setIdleMode(IdleMode.kBrake);

    // Set motor ramp rate to 0.25 seconds //
    m_intake.setOpenLoopRampRate(IntakeConstants.RAMP_RATE_IN_SEC);
  }

  /**
   * Set the intake to intake or outtake
   * @param intake true to intake, false to outtake
   */
  public void setIntake(boolean intake) {
    if (intake) {
      m_intake.set(1.0);
    } else {
      m_intake.set(-1.0);
    }
  } 

  /**
   * Set the intake to a specific speed
   * @param speed the speed to set the intake to
   */
  public void set(double speed) {
    m_intake.set(speed);
  }

  /**
   * Set the intake to intake at 100% speed
   */
  public void intake100() {
    m_intake.set(1.0);
  }

  /**
   * Set the intake to intake at 40% speed
   */
  public void intake40() {
    m_intake.set(0.4);
  }

  /**
   * Set the intake to outtake at 100% speed
   */
  public void outtake100() {
    m_intake.set(-1.0);
  }

  /**
   * Set the intake to outtake at 40% speed
   */
  public void outtake40() {
    m_intake.set(-0.4);
  }

  /**
   * Stop the intake
   */
  public void stopIntake() {
    m_intake.set(0.0);
  }
}
