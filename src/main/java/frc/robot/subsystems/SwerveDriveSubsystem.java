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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveDriveSubsystem extends SubsystemBase {

  public final SwerveDrive swerve;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(SwerveConstants.MAX_SPEED_METERS_PER_SECOND);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(SwerveConstants.MAX_SPEED_METERS_PER_SECOND);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(SwerveConstants.MAX_SPEED_METERS_PER_SECOND);

  /** Creates a new SwerveDriveSubsystem. */
  public SwerveDriveSubsystem() {
    /* 
     * Initialize the swerve drive based on the
     * configuration files deployed to the RoboRIO
     */
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    // SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
    
    // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
    //  In this case the gear ratio is 12.8 motor revolutions per wheel rotation.
    //  The encoder resolution per motor revolution is 1 per motor revolution.
    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8, 1);

    // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO * ENCODER RESOLUTION).
    //  In this case the wheel diameter is 4 inches, which must be converted to meters to get meters/second.
    //  The gear ratio is 6.75 motor revolutions per wheel rotation.
    //  The encoder resolution per motor revolution is 1 per motor revolution.
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 8.14, 1);

    try {
      swerve =
          new SwerveParser(
            new File(
              Filesystem.getDeployDirectory(), "swerve"
            )
          ).createSwerveDrive(
            SwerveConstants.MAX_SPEED_METERS_PER_SECOND,
            angleConversionFactor,
            driveConversionFactor
          );

      swerve.setHeadingCorrection(true);

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
            new Translation2d(translationVal,strafeVal).times(SwerveConstants.MAX_SPEED_METERS_PER_SECOND),
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

  public Command driveHeading(
    DoubleSupplier translationSup,
    DoubleSupplier strafeSup,
    DoubleSupplier headingXSup,
    DoubleSupplier headingYSup
  ) {
      return run(() -> {
      double translation = translationSup.getAsDouble();
      double strafe = strafeSup.getAsDouble();
      double headingX = headingXSup.getAsDouble();
      double headingY = headingYSup.getAsDouble();

      double translationVal =
          translationLimiter.calculate(
              MathUtil.applyDeadband(
                  translation, Constants.GeneralConstants.swerveDeadband));
      double strafeVal =
          strafeLimiter.calculate(
              MathUtil.applyDeadband(
                  strafe, Constants.GeneralConstants.swerveDeadband));

      ChassisSpeeds desiredSpeeds
          = swerve.swerveController.getTargetSpeeds(
              translationVal * SwerveConstants.MAX_SPEED_METERS_PER_SECOND,
              strafeVal * SwerveConstants.MAX_SPEED_METERS_PER_SECOND,
              headingX,
              headingY,
              swerve.getYaw().getRadians(),
              SwerveConstants.MAX_SPEED_METERS_PER_SECOND
          );

      driveHeading(desiredSpeeds);
    })
    .withName("TeleopHeadingSwerve");
  }

  public void driveHeading(ChassisSpeeds velocity)
  {
    swerve.driveFieldOriented(velocity); // Open loop is disabled since it shouldn't be used most of the time.
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
