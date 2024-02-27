// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants.GeneralConstants.RobotMode;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  // Create new Shuffleboard tab for the intake subsystem //
  private final ShuffleboardTab m_intakShuffleboardTab = Shuffleboard.getTab("Intake");
  private GenericEntry m_intakePowerPercentageEntry = null;
  private GenericEntry m_intakePowerVoltageEntry = null;
  private GenericEntry m_intakePowerCurrentEntry = null;
  private GenericEntry m_intakePowerTemperatureEntry = null;
  private GenericEntry m_intakePowerRPMEntry = null;

  public GenericEntry m_intakePowerPercentageSetEntry = null;

  // Intake Motor Controller //
  private final CANSparkMax m_intake = new CANSparkMax(IntakeConstants.MOTOR_ID, MotorType.kBrushless);

  // Intake Beambreak Sensor //
  // private final DigitalInput m_intakeBeamBreak = new DigitalInput(IntakeConstants.BEAM_BREAK_DIGITAL_CHANNEL);

  /**
   * Creates a new IntakeSubsystem.
   */
  public IntakeSubsystem() {
    // Restore motor to factory default //
    m_intake.restoreFactoryDefaults();

    // Set the motor to only use 8 volts of power //
    m_intake.enableVoltageCompensation(IntakeConstants.NOMINAL_VOLTAGE);

    // Set the motor to only use 20 amp stall limit, 10 amp free limit, and limit to 3k RPM //
    m_intake.setSmartCurrentLimit(
        IntakeConstants.STALL_CURRENT_LIMIT,
        IntakeConstants.FREE_CURRENT_LIMIT
    );

    // Set the motor's hard stop current limit to 40 amps //
    m_intake.setSecondaryCurrentLimit(IntakeConstants.SECONDARY_CURRENT_LIMIT);

    // Set the motor to brake mode //
    m_intake.setIdleMode(IdleMode.kBrake);

    // Set motor ramp rate to 0.25 seconds //
    m_intake.setOpenLoopRampRate(IntakeConstants.RAMP_RATE_IN_SEC);

    // Create Shuffleboard entries for the IntakeSubsystem if the robot is in test mode //
    if (GeneralConstants.CURRENT_MODE == RobotMode.TEST) {
      m_intakePowerPercentageEntry = m_intakShuffleboardTab.add("Intake Power Percentage Reading", m_intake.getAppliedOutput()).getEntry();
      m_intakePowerVoltageEntry = m_intakShuffleboardTab.add("Intake Power Voltage Reading", m_intake.getBusVoltage()).getEntry();
      m_intakePowerCurrentEntry = m_intakShuffleboardTab.add("Intake Power Current Reading", m_intake.getOutputCurrent()).getEntry();
      m_intakePowerTemperatureEntry = m_intakShuffleboardTab.add("Intake Power Temperature Reading", m_intake.getMotorTemperature()).getEntry();
      m_intakePowerRPMEntry = m_intakShuffleboardTab.add("Intake Power RPM Reading", m_intake.getEncoder().getVelocity()).getEntry();

      m_intakePowerPercentageSetEntry
        = m_intakShuffleboardTab.add("Intake Power Percentage Setting", 0)
          .withWidget(BuiltInWidgets.kNumberSlider)
          .getEntry();
    }
  }

  public void setSpeedForward100() {
    m_intake.set(1);
  }

  public void setSpeedReverse100() {
    m_intake.set(-1);
  }

  public void setSpeed0() {
    m_intake.set(0);
  }

  // /**
  //  * Get the state of the beam break sensor
  //   * @return true if the beam break sensor is broken, false if it is not
  //  */
  // public boolean getBeamBreak() {
  //   return m_intakeBeamBreak.get();
  // }

  /**
   * Set the intake to intake or outtake
   * @param intake true to intake, false to outtake
   */
  public void setIntake(boolean intake) {
    if (intake) {
      m_intake.set(0.3);
    } else {
      m_intake.set(-0.3);
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

  /**
   * Get the current of the intake motor
   */
  public double getCurrent() {
    return m_intake.getOutputCurrent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(GeneralConstants.CURRENT_MODE == RobotMode.TEST) {
      m_intakePowerPercentageEntry.setDouble(m_intake.getAppliedOutput());
      m_intakePowerVoltageEntry.setDouble(m_intake.getBusVoltage());
      m_intakePowerCurrentEntry.setDouble(m_intake.getOutputCurrent());
      m_intakePowerTemperatureEntry.setDouble(m_intake.getMotorTemperature());
      m_intakePowerRPMEntry.setDouble(m_intake.getEncoder().getVelocity());
    }
  }
}
