// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.UptakeSubsystem;

public class UptakeForward100Command extends Command {
  private UptakeSubsystem m_uptakeSubsystem;

  /** Creates a new Uptake100Command. */
  public UptakeForward100Command(UptakeSubsystem uptakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_uptakeSubsystem = uptakeSubsystem;
    addRequirements(m_uptakeSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_uptakeSubsystem.setSpeedForward100();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_uptakeSubsystem.setSpeed0();
  }
}
