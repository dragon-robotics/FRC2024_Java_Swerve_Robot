// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.KitBotConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.KitBotVariableCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.KitBotSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.commands.KitBotVariableCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  public final KitBotSubsystem m_kitBotSubsystem = new KitBotSubsystem();
  public final LEDSubsystem m_ledSubsystem = new LEDSubsystem();

  // Define Driver and Operator controllers //
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  // private final SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Init Auto Chooser //
    // autoChooser = AutoBuilder.buildAutoChooser("OnePieceExit");
    // SmartDashboard.putData("OnePieceExit", autoChooser);


    // Register Named Commands //
    NamedCommands.registerCommand("OnePieceExit", getAutonomousCommand());
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

    // Set default teleop command to drive //
    // m_swerveDriveSubsystem.setDefaultCommand(
    //   m_swerveDriveSubsystem.drive(
    //     () -> -m_driverController.getLeftY(),   // Translation
    //     () -> -m_driverController.getLeftX(),   // Strafe
    //     () -> -m_driverController.getRightX()   // Rotation
    //   )
    // );

    m_kitBotSubsystem.setDefaultCommand(
      new KitBotVariableCommand(m_kitBotSubsystem, () -> -m_operatorController.getLeftY(), () -> -m_operatorController.getRightX())
    );



    // m_swerveDriveSubsystem.setDefaultCommand(
    //   m_swerveDriveSubsystem.driveHeading(
    //     () -> -m_driverController.getLeftY(),   // Translation
    //     () -> -m_driverController.getLeftX(),   // Strafe
    //     () -> -m_driverController.getRightX(),  // X component of angle
    //     () -> -m_driverController.getRightY()   // Y component of angle
    //   )

    // );

    // m_operatorController.a().whileTrue(Commands.run(() -> m_kitBotSubsystem.shootADummy()));
    // m_operatorController.b().whileTrue(Commands.run(() -> m_kitBotSubsystem.shootBDummy()));
    // m_operatorController.x().whileTrue(Commands.run(() -> m_kitBotSubsystem.shootXDummy()));
    // m_operatorController.y().whileTrue(Commands.run(() -> m_kitBotSubsystem.shootYDummy()));

    m_operatorController.a().whileTrue(Commands.run(() -> m_kitBotSubsystem.armExtendoForward()));
    m_operatorController.b().whileTrue(Commands.run(() -> m_kitBotSubsystem.armExtendoBackward()));

    // Use the "Back" button to reset the Gyro orientation //
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
