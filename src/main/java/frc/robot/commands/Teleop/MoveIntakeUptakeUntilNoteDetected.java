// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.UptakeSubsystem;

public class MoveIntakeUptakeUntilNoteDetected extends Command {

  private IntakeSubsystem m_intake;
  private UptakeSubsystem m_uptake;
  private DoubleSupplier m_intakeSpeed;
  private DoubleSupplier m_uptakeSpeed;

  private Thread thread;

  // @TODO: 1. Make sure to clean the sensors
  // @TODO: 2. Make sure to position the sensors differently
  // @TODO: 3. Use the 5mm sensor instead
  // @TODO: 4. Use threads to speed up update rate

  /** Creates a new MoveUptakeUntilNoteDetected. */
  public MoveIntakeUptakeUntilNoteDetected(
    IntakeSubsystem intake,
    UptakeSubsystem uptake,
    DoubleSupplier intakeSpeed,
    DoubleSupplier uptakeSpeed) {

    m_intake = intake;
    m_uptake = uptake;
    m_uptakeSpeed = uptakeSpeed;
    m_intakeSpeed = intakeSpeed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    addRequirements(uptake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // thread = new Thread(() -> {
    //   while (!Thread.interrupted()) {
    //     // Command update logic here
    //     m_intake.set(m_intakeSpeed.getAsDouble());
    //     m_uptake.set(m_uptakeSpeed.getAsDouble());

    //     // If the note is detected, break the loop
    //     if(!m_uptake.isNoteDetected()) break;

    //     try {
    //       Thread.sleep(10); // Sleep for 10ms, for a 100Hz update rate
    //     } catch (InterruptedException e) {
    //       break;
    //     }
    //   }
    // });

    // thread.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.set(m_intakeSpeed.getAsDouble());
    m_uptake.set(m_uptakeSpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // if (thread != null) {
    //   thread.interrupt();
    // }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_uptake.isNoteDetected();
    // return !thread.isAlive();
  }
}
