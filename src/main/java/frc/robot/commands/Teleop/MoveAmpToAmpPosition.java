// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Teleop;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.AmpConstants;
import frc.robot.subsystems.AmpSmartMotionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveAmpToAmpPosition extends ProfiledPIDCommand {
  /** Creates a new MoveAmpToAmpPositionPID. */
  public MoveAmpToAmpPosition(AmpSmartMotionSubsystem amp) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            0.1,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(0.1, 0.1)),
        // This should return the measurement
        () -> amp.getAmpPosition(),
        // This should return the goal (can also be a constant)
        () -> new TrapezoidProfile.State(AmpConstants.AMP_SETPOINT, 0),
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          amp.setAmpPosition(setpoint.position);
        },
        amp);

    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(0.05);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
