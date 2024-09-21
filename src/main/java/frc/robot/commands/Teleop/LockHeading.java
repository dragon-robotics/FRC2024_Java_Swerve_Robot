package frc.robot.commands.Teleop;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class LockHeading extends Command {
  private final SwerveDriveSubsystem m_driveSubsystem;
  private final double m_desiredAngle;

  public LockHeading(SwerveDriveSubsystem driveSubsystem, double desiredAngle) {
    m_driveSubsystem = driveSubsystem;
    m_desiredAngle = desiredAngle;

    // Use requires() here to declare subsystem dependencies
    addRequirements(m_driveSubsystem);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    double currentAngle = m_driveSubsystem.getYaw();

    // Calculate the difference between the desired and current angles
    double error = m_desiredAngle - currentAngle;

    // Adjust the error to use the shortest path, taking into account the 360°/0° boundary
    error = ((error + 180) % 360) - 180;

    // Use a simple proportional controller to adjust the robot's heading
    double correction = 0.01 * error;

    ChassisSpeeds desiredSpeeds
          = m_driveSubsystem.swerve.swerveController.getTargetSpeeds(
              0,
              0,
              correction,
              m_driveSubsystem.swerve.getYaw().getRadians(),
              SwerveConstants.MAX_SPEED_METERS_PER_SECOND
          );

    m_driveSubsystem.driveHeading(desiredSpeeds);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return Math.abs(m_desiredAngle - m_driveSubsystem.getYaw()) < 1.0;
  }
}