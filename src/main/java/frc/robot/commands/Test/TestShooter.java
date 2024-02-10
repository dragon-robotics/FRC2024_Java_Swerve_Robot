// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSmartVelocitySubsystem;

public class TestShooter extends Command {
  private final ShooterSmartVelocitySubsystem m_shooter;

  /** Creates a new TestShooter. */
  public TestShooter(ShooterSmartVelocitySubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);

    m_shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = m_shooter.m_shooterLeadPowerPercentageSetEntry.getDouble(0);
    if (speed < -0.7)
      speed = -0.7;
    else if (speed > 0.7)
      speed = 0.7; 
    m_shooter.set(speed);
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
