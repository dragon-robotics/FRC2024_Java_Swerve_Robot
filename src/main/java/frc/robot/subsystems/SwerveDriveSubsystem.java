// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveDriveSubsystem extends SubsystemBase {

  private final SwerveDrive swerve;

  private SwerveDriveKinematics kinematics;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(Units.feetToMeters(14.5));
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(Units.feetToMeters(14.5));
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(Units.feetToMeters(14.5));

  /** Creates a new SwerveDriveSubsystem. */
  public SwerveDriveSubsystem() {
    /* 
     * Initialize the swerve drive based on the
     * configuration files deployed to the RoboRIO
     */
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerve =
          new SwerveParser(
            new File(
              Filesystem.getDeployDirectory(), "swerve"
            )
          ).createSwerveDrive(
            Units.feetToMeters(SwerveConstants.MAX_SPEED)
          );

      // Configure the AutoBuilder //
      AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetOdometry,
        this::getRobotRelativeSpeeds,
        this::driveRobotRelative,
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> Boolean.FALSE,
        this);
        
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

  }

  /**
   * Drive Command
   *
   * @return the drive command
   */
  public Command drive(
    DoubleSupplier translationSup,
    DoubleSupplier strafeSup,
    DoubleSupplier rotationSup
  ) {
    return run(() -> {
        double translation = translationSup.getAsDouble();
        double strafe = strafeSup.getAsDouble();
        double rotation = rotationSup.getAsDouble();
        double translationVal =
            translationLimiter.calculate(
                MathUtil.applyDeadband(
                    translation, Constants.GeneralConstants.swerveDeadband));
        double strafeVal =
            strafeLimiter.calculate(
                MathUtil.applyDeadband(
                    strafe, Constants.GeneralConstants.swerveDeadband));
        double rotationVal =
            rotationLimiter.calculate(
                MathUtil.applyDeadband(
                    rotation, Constants.GeneralConstants.swerveDeadband));

        drive(
            new Translation2d(translationVal, strafeVal).times(SwerveConstants.MAX_SPEED),
            rotationVal * swerve.swerveController.config.maxAngularVelocity,
            true,
            false);
      })
      .withName("TeleopSwerve");
  }

  public void drive(
    Translation2d translationVal, double rotationVal, boolean fieldRelative, boolean openLoop) {
    swerve.drive(translationVal, rotationVal, fieldRelative, openLoop);
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    swerve.setChassisSpeeds(speeds);
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorIdleMode(brake);
  }

  public void zeroGyro() {
    swerve.zeroGyro();
  }

  public void lock() {
    swerve.lockPose();
  }

  public double getYaw() {
    return swerve.getYaw().getDegrees();
  }

  public double getPitch() {
    return swerve.getPitch().getDegrees();
  }

  public void resetOdometry(Pose2d pose) {
    swerve.resetOdometry(pose);
  }

  public Pose2d getPose() {
    return swerve.getPose();
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return swerve.getRobotVelocity();
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    swerve.drive(targetSpeeds);
  }

  @Override
  public void periodic() {
    // Update the robot odometry per scheduler run //
    swerve.updateOdometry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
