// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private Spark m_ledController0;

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    m_ledController0= new Spark(0);
  }

  public void setStaticColor() {
    m_ledController0.set(0.87);
  }
}
