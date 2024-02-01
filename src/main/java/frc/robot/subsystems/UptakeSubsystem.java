// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UptakeSubsystem extends SubsystemBase {
  /** Creates a new UptakeSubsystem. */
  
  CANSparkMax m_uptakeLead = new CANSparkMax(0, MotorType.kBrushless);
  CANSparkMax m_uptakeFollow = new CANSparkMax(1, MotorType.kBrushless);

  public UptakeSubsystem() {
    m_uptakeLead.restoreFactoryDefaults();
    m_uptakeFollow.restoreFactoryDefaults();

    m_uptakeFollow.follow(m_uptakeLead);

    m_uptakeLead.set(0);
  }

  public void setSpeedFoward100() {
    m_uptakeLead.set(1);
  }

  public void setSpeedReverse100() {
    m_uptakeLead.set(-1);
  }

  public void setSpeed0() {
    m_uptakeLead.set(0);
  }
}
