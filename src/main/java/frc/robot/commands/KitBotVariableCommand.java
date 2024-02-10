// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KitBotSubsystem;

public class KitBotVariableCommand extends Command {
  /** Creates a new KitBotVariableCommand. */
  private KitBotSubsystem m_kitBotSubsystem;
  private DoubleSupplier m_yAxisLeft;
  private DoubleSupplier m_yAxisRight;

  public KitBotVariableCommand(KitBotSubsystem kitBotSubsystem, DoubleSupplier yAxisLeft, DoubleSupplier yAxisRight) {
    m_kitBotSubsystem = kitBotSubsystem;
    m_yAxisLeft = yAxisLeft;
    m_yAxisRight = yAxisRight;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_kitBotSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_kitBotSubsystem.variableKitBotSpeed(m_yAxisLeft.getAsDouble());
    // m_kitBotSubsystem.stop();
    m_kitBotSubsystem.setAGroup(m_yAxisLeft.getAsDouble());
    m_kitBotSubsystem.setBGroup(m_yAxisRight.getAsDouble());
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
