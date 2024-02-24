// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.UptakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeNoteToUptake extends ParallelDeadlineGroup {
  /** Creates a new IntakeNoteToUptake. */
  public IntakeNoteToUptake(IntakeSubsystem intake, UptakeSubsystem uptake, DoubleSupplier speed) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new MoveUptake(uptake, speed));
    addCommands(new MoveIntake(intake, speed));
    // addCommands(new FooCommand(), new BarCommand());
  }
}
