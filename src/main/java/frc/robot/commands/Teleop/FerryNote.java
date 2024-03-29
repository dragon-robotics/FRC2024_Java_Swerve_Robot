// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.UptakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FerryNote extends SequentialCommandGroup {
  /** Creates a new FerryNote. */
  public FerryNote(
    IntakeSubsystem intake,  
    UptakeSubsystem uptake,
    ArmSubsystem arm,
    ShooterSubsystem shooter
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveArmToPos(arm, ArmConstants.SHOOTER_GOAL),
      new WaitCommand(0.5),
      new MoveShooter(shooter, () -> -0.7).withTimeout(0.5),
      new MoveIntakeUptakeUntilNoteDetected(
        intake, uptake, ()-> -0.6, () -> -0.4)
        .deadlineWith(new MoveShooter(shooter, () -> -0.7)),
      new MoveShooter(shooter, () -> -0.7).withTimeout(1.0)
    );
  }
}
