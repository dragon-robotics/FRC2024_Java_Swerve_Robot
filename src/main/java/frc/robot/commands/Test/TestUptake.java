// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.UptakeSubsystem;

public class TestUptake extends Command {
  private final UptakeSubsystem m_uptake;

  /** Creates a new TestUptake. */
  public TestUptake(UptakeSubsystem uptake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(uptake);

    m_uptake = uptake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Limit Power to 70% //
    double speed = m_uptake.m_uptakeLeadPowerPercentageSetEntry.getDouble(0);
    if (speed < -0.7)
      speed = -0.7;
    else if (speed > 0.7)
      speed = 0.7;
    m_uptake.set(speed);
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
