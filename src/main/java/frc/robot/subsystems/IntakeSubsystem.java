// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  
  CANSparkMax m_intakeLead = new CANSparkMax(0, MotorType.kBrushless);
  CANSparkMax m_intakeFollow = new CANSparkMax(1, MotorType.kBrushless);

  public IntakeSubsystem() {
    m_intakeFollow.follow(m_intakeLead);
  }

  public void intake100() {
    m_intakeLead.set(1);
  }

  public void intake40() {
    m_intakeLead.set(0.4);
  }
}
