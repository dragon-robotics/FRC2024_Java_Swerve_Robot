// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Test;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpSmartMotionSubsystem;

public class TestAmpSetpoints extends Command {
  private final AmpSmartMotionSubsystem m_amp;
  private final DoubleSupplier m_power;

  /** Creates a new TestAmpSetpoints. */
  public TestAmpSetpoints(AmpSmartMotionSubsystem amp, DoubleSupplier power) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(amp);

    m_amp = amp;
    m_power = power;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = m_power.getAsDouble();
    if (m_power.getAsDouble() < -0.2)
      speed = -0.2;
    else if (m_power.getAsDouble() > 0.2)
      speed = 0.2;
    m_amp.setAmpSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
