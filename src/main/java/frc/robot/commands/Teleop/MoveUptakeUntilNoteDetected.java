// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.UptakeSubsystem;

public class MoveUptakeUntilNoteDetected extends Command {

  private UptakeSubsystem m_uptake;
  private DoubleSupplier m_speed;

  /** Creates a new MoveUptakeUntilNoteDetected. */
  public MoveUptakeUntilNoteDetected(UptakeSubsystem uptake, DoubleSupplier speed) {

    m_uptake = uptake;
    m_speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(uptake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_uptake.set(m_speed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_uptake.isNoteDetected();
  }
}
