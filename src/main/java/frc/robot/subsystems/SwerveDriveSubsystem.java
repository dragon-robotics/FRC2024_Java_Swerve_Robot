// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.BooleanSupplier;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveDriveSubsystem extends SubsystemBase {

  public final SwerveDrive swerve;

  private final SlewRateLimiter translationLimiter = new SlewRateLimiter(SwerveConstants.MAX_SPEED_METERS_PER_SECOND);
  private final SlewRateLimiter strafeLimiter = new SlewRateLimiter(SwerveConstants.MAX_SPEED_METERS_PER_SECOND);
  private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(SwerveConstants.MAX_SPEED_METERS_PER_SECOND);

  // LED Controller //
  private final Spark m_ledController = new Spark(ShooterConstants.LED_CHANNEL);

  /** Creates a new SwerveDriveSubsystem. */
  public SwerveDriveSubsystem() {
    /* 
     * Initialize the swerve drive based on the
     * configuration files deployed to the RoboRIO
     */
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    // SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    // SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.NONE;
    
    // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
    //  In this case the gear ratio is 12.8 motor revolutions per wheel rotation.
    //  The encoder resolution per motor revolution is 1 per motor revolution.
    double angleConversionFactor
        = SwerveMath.calculateDegreesPerSteeringRotation(
              SwerveConstants.ANGLE_GEAR_RATIO,
              SwerveConstants.PULSE_PER_ROTATION
          );

    // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO * ENCODER RESOLUTION).
    //  In this case the wheel diameter is 4 inches, which must be converted to meters to get meters/second.
    //  The gear ratio is 6.75 motor revolutions per wheel rotation.
    //  The encoder resolution per motor revolution is 1 per motor revolution.
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(
        SwerveConstants.WHEEL_DIAMETER_METERS,
        SwerveConstants.DRIVE_GEAR_RATIO,
        SwerveConstants.PULSE_PER_ROTATION
    );

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

      // swerve.setHeadingCorrection(true);
      // swerve.setCosineCompensator(true);

      // Configure the AutoBuilder //
      setupPathPlanner();
        
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

  }

  /**
   * Setup AutoBuilder for PathPlanner.
   */
  public void setupPathPlanner()
  {
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                                        new PIDConstants(5.0, 0.0, 0.0),
                                        // Translation PID constants
                                        new PIDConstants(swerve.swerveController.config.headingPIDF.p,
                                                          swerve.swerveController.config.headingPIDF.i,
                                                          swerve.swerveController.config.headingPIDF.d),
                                        // Rotation PID constants
                                        4.5,
                                        // Max module speed, in m/s
                                        swerve.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                                        // Drive base radius in meters. Distance from robot center to furthest module.
                                        new ReplanningConfig()
                                        // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  /**
   * Drive Command
   *
   * @return the drive command
   */
  public Command drive(
    DoubleSupplier translationSup,
    DoubleSupplier strafeSup,
    DoubleSupplier rotationSup,
    BooleanSupplier halfSpeedSup
  ) {
    return run(() -> {
        double translation = translationSup.getAsDouble();
        double strafe = strafeSup.getAsDouble();
        double rotation = rotationSup.getAsDouble();
        Boolean halfSpeed = halfSpeedSup.getAsBoolean();

        // Apply half speed if the half speed button is pressed
        if(halfSpeed)
        {
          m_ledController.set(LEDConstants.TWINKLES_LAVA_PALETTE);

          translation *= 0.5;
          strafe *= 0.5;
          rotation *= 0.5;
        }

        double translationVal =
          translationLimiter.calculate(
            MathUtil.applyDeadband(
              translation, Constants.SwerveConstants.SWERVE_DEADBAND));
        double strafeVal =
          strafeLimiter.calculate(
            MathUtil.applyDeadband(
              strafe, Constants.SwerveConstants.SWERVE_DEADBAND));
        double rotationVal =
            rotationLimiter.calculate(
                MathUtil.applyDeadband(
                    rotation, Constants.SwerveConstants.SWERVE_DEADBAND));

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
                  translation, Constants.SwerveConstants.SWERVE_DEADBAND));
      double strafeVal =
          strafeLimiter.calculate(
              MathUtil.applyDeadband(
                  strafe, Constants.SwerveConstants.SWERVE_DEADBAND));

      ChassisSpeeds desiredSpeeds
          = swerve.swerveController.getTargetSpeeds(
              translationVal,
              strafeVal,
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

  public void setLED(double value) {
    m_ledController.set(value);
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
