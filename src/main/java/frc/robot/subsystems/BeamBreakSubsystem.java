// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeamBreakSubsystem extends SubsystemBase {
  /** Creates a new BeamBreakSubsystem. */
  DigitalInput beamBrake0 = new DigitalInput(0);

  public BeamBreakSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println(beamBrake0.get());
  }
}
