// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.GeneralConstants.RobotMode;
// import frc.robot.Constants.GeneralConstants.RobotMode;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Test.TestArmSetpoints;
import frc.robot.commands.Test.TestClimber;
import frc.robot.commands.Test.TestIntake;
import frc.robot.commands.Test.TestShooter;
import frc.robot.commands.Test.TestUptake;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BeamBreakSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.UptakeSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // The robot's subsystems and commands are defined here...
  // public final SwerveDriveSubsystem m_swerveDriveSubsystem = new SwerveDriveSubsystem();

  // public final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  public final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public final ShooterSubsystem m_shooterSmartVelocitySubsystem = new ShooterSubsystem();
  public final ArmSubsystem m_armSmartMotionSubsystem = new ArmSubsystem();
  public final UptakeSubsystem m_uptakeSubsystem = new UptakeSubsystem();
  public final BeamBreakSubsystem m_beamBreakSubsystem = new BeamBreakSubsystem();

  // Define Driver and Operator controllers //
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.DRIVER_PORT);

  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.OPERATOR_PORT);

  // private final SendableChooser<Command> autoChooser;

  // Create all the shuffleboard tab for testing //
  public ShuffleboardTab m_testShuffleboardTab = null;
  public ShuffleboardTab m_intakeShuffleboardTab = null;
  public ShuffleboardTab m_uptakeShuffleboardTab = null;
  public ShuffleboardTab m_shooterShuffleboardTab = null;
  public ShuffleboardTab m_ampShuffleboardTab = null;
  public ShuffleboardTab m_climberShuffleboardTab = null;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // // Init Auto Chooser //
    // autoChooser = AutoBuilder.buildAutoChooser("OnePieceExit");
    // SmartDashboard.putData("OnePieceExit", autoChooser);

    // // Register Named Commands //
    // NamedCommands.registerCommand("OnePieceExit", getAutonomousCommand());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // // Set default teleop command to drive //
    // m_swerveDriveSubsystem.setDefaultCommand(
    //   m_swerveDriveSubsystem.drive(
    //     () -> -m_driverController.getLeftY(),   // Translation
    //     () -> -m_driverController.getLeftX(),   // Strafe
    //     () -> -m_driverController.getRightX()   // Rotation
    //   )
    // );

    // m_swerveDriveSubsystem.setDefaultCommand(
    //   m_swerveDriveSubsystem.driveHeading(
    //     () -> -m_driverController.getLeftY(),   // Translation
    //     () -> -m_driverController.getLeftX(),   // Strafe
    //     () -> -m_driverController.getRightX(),  // X component of angle
    //     () -> -m_driverController.getRightY()   // Y component of angle
    //   )
    // );

    if (GeneralConstants.CURRENT_MODE == RobotMode.TEST){
      // m_climberSubsystem.setDefaultCommand(new TestClimber(m_climberSubsystem));
      m_intakeSubsystem.setDefaultCommand(new TestIntake(m_intakeSubsystem, () -> -m_operatorController.getRightY()));
      m_uptakeSubsystem.setDefaultCommand(new TestUptake(m_uptakeSubsystem, () -> -m_operatorController.getLeftY()));
      m_armSmartMotionSubsystem.setDefaultCommand(Commands.run(() -> m_armSmartMotionSubsystem.setArmSpeed(0.0), m_armSmartMotionSubsystem));
      m_shooterSmartVelocitySubsystem.setDefaultCommand(new TestShooter(m_shooterSmartVelocitySubsystem, () -> -m_operatorController.getRightTriggerAxis(), () -> -m_operatorController.getLeftTriggerAxis()));

      m_operatorController.a()
          .whileTrue(Commands.run(() -> m_armSmartMotionSubsystem.setArmSpeed(0.1), m_armSmartMotionSubsystem));

      m_operatorController.b()
          .whileTrue(Commands.run(() -> m_armSmartMotionSubsystem.setArmSpeed(-0.1), m_armSmartMotionSubsystem));
    } else {
      // m_climberSubsystem.setDefaultCommand(new TestClimber(m_climberSubsystem));
      m_intakeSubsystem.setDefaultCommand(Commands.run(() -> m_intakeSubsystem.stopIntake(), m_intakeSubsystem));
      m_uptakeSubsystem.setDefaultCommand(Commands.run(() -> m_uptakeSubsystem.stopUptake(), m_uptakeSubsystem));
      m_armSmartMotionSubsystem.setDefaultCommand(Commands.run(() -> m_armSmartMotionSubsystem.stopArm(), m_armSmartMotionSubsystem));
      m_shooterSmartVelocitySubsystem.setDefaultCommand(Commands.run(() -> m_shooterSmartVelocitySubsystem.stopShooter(), m_shooterSmartVelocitySubsystem));

      m_operatorController.a()
          .whileTrue(Commands.run(() -> m_armSmartMotionSubsystem.setArmSpeed(0.1), m_armSmartMotionSubsystem));

      m_operatorController.b()
          .whileTrue(Commands.run(() -> m_armSmartMotionSubsystem.setArmSpeed(-0.1), m_armSmartMotionSubsystem));

      // Intake to Uptake - User should hold the button //

      // Intake Amp Sequence - User should hold the button //

      // Prepare to Shoot Speaker - User should hold the button //

      // Score Speaker - User should hold the button //

      // Score Amp - User should hold the button //
    }

    // Use the "A" button to reset the Gyro orientation //
    // m_driverController.a().onTrue(Commands.runOnce(() -> m_swerveDriveSubsystem.zeroGyro()));

    // // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    // return autoChooser.getSelected();
    return null;
  }
}
