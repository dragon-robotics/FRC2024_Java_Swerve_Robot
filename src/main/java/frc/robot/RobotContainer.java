// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.CustomButtonBoxConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.GeneralConstants.RobotMode;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Teleop.MoveArmToPos;
import frc.robot.commands.Teleop.HoldArmToPosition;
import frc.robot.commands.Teleop.LockHeading;
import frc.robot.commands.Teleop.MoveArmToInitialPosition;
import frc.robot.commands.Teleop.MoveArmToShootPosition;
import frc.robot.commands.Teleop.MoveIntake;
import frc.robot.commands.Teleop.MoveIntakeAdjPercent;
import frc.robot.commands.Teleop.MoveIntakeUntilNoteDetected;
import frc.robot.commands.Teleop.MoveShooter;
import frc.robot.commands.Teleop.MoveShooterAdjPercent;
import frc.robot.commands.Teleop.MoveUptake;
import frc.robot.commands.Teleop.MoveUptakeAdjPercent;
import frc.robot.commands.Teleop.ScoreAmp;
import frc.robot.commands.Teleop.MoveIntakeUptakeUntilNoteDetected;
import frc.robot.commands.Test.TestArmSetpoints;
import frc.robot.commands.Test.TestClimber;
import frc.robot.commands.Test.TestIntake;
import frc.robot.commands.Test.TestShooter;
import frc.robot.commands.Test.TestUptake;
import frc.robot.commands.Test.TuneUptakeAmpShot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.UptakeSubsystem;
import frc.robot.subsystems.Limelight3Subsystem;

import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.revrobotics.CANSparkMax;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final SwerveDriveSubsystem m_swerveDriveSubsystem = new SwerveDriveSubsystem();

  public final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  public final UptakeSubsystem m_uptakeSubsystem = new UptakeSubsystem();
  public final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  public final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  public final LEDSubsystem m_ledSubsystem = new LEDSubsystem();
  public final Limelight3Subsystem m_limelight3Subsystem = new Limelight3Subsystem();

  // Define Driver and Operator controllers //
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.DRIVER_PORT);
  
  private final XboxController m_driverControllerRaw = 
      new XboxController(OperatorConstants.DRIVER_PORT);

  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.OPERATOR_PORT);

  private final CommandJoystick m_operatorButtonBoxController =
      new CommandJoystick(OperatorConstants.OPERATOR_BUTTON_PORT);

  private final Joystick m_testController =
      new Joystick(OperatorConstants.TEST_PORT);

  private final CommandJoystick m_testCommandJoystick =
      new CommandJoystick(OperatorConstants.TEST_PORT);

//   // Define Driver and Operator controllers - Temporary fix for forgetting Xbox Controller //
//   private final CommandJoystick m_driverController =
//       new CommandJoystick(OperatorConstants.DRIVER_PORT);

//   private final CommandJoystick m_operatorController =
//       new CommandJoystick(OperatorConstants.OPERATOR_PORT);


  private final SendableChooser<Command> autoChooser;

  // Create all the shuffleboard tab for testing //
  public ShuffleboardTab m_testShuffleboardTab = null;
  public ShuffleboardTab m_intakeShuffleboardTab = null;
  public ShuffleboardTab m_uptakeShuffleboardTab = null;
  public ShuffleboardTab m_shooterShuffleboardTab = null;
  public ShuffleboardTab m_ampShuffleboardTab = null;
  public ShuffleboardTab m_climberShuffleboardTab = null;

  GenericEntry noteIsInIntakeEntry = Shuffleboard.getTab("SmartDashboard")
      .add("NoteIsInIntake", false)
      .withWidget("Boolean Box")
      .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "maroon"))
      .getEntry();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Register Named Commands //
    NamedCommands.registerCommand("MoveUptake0", new MoveUptake(m_uptakeSubsystem, () -> 0.0));
    NamedCommands.registerCommand("MoveUptake100", new MoveUptake(m_uptakeSubsystem, () -> -1.0));    
    NamedCommands.registerCommand("MoveIntake0", new MoveIntake(m_intakeSubsystem, () -> 0.0));
    NamedCommands.registerCommand("MoveIntake100", new MoveIntake(m_intakeSubsystem, () -> -1.0));
    NamedCommands.registerCommand(
      "MoveIntakeUntilNoteDetected",
      new MoveIntakeUntilNoteDetected(m_intakeSubsystem, () -> -0.7)
              .andThen(new MoveIntake(m_intakeSubsystem, () -> 0.0).withTimeout(0.25))
              .andThen(Commands.runOnce(() -> m_ledSubsystem.set(LEDConstants.ORANGE))));
    NamedCommands.registerCommand(
      "UptakeShoot",
      new MoveUptake(m_uptakeSubsystem, () -> -1.0).withTimeout(1.0)
      .andThen(new MoveIntake(m_intakeSubsystem, () -> -0.5)));
    

    // Init Auto Chooser //
    autoChooser = AutoBuilder.buildAutoChooser("UptakeShoot");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the trigger bindings
    configureBindings();
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

    //#region Xbox Controller Bindings

    // Set default teleop command to drive //
    m_swerveDriveSubsystem.setDefaultCommand(
      m_swerveDriveSubsystem.drive(
        () -> -m_driverController.getLeftY(),   // Translation
        () -> -m_driverController.getLeftX(),   // Strafe
        () -> -m_driverController.getRightX(),
        //  + (m_limelight3Subsystem.alignHorizontal(LimelightConstants.HORIZONTAL_KP)
        //    * m_driverControllerRaw.getRawAxis(JoystickConstants.TRIGGER_RIGHT)),  // Rotation
        () -> m_driverController.getHID().getRightBumper()  // Half-Speed
      ).alongWith(Commands.runOnce(() -> m_ledSubsystem.set(LEDConstants.BLACK)))
    );

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
      m_armSubsystem.setDefaultCommand(Commands.run(() -> m_armSubsystem.setArmSpeed(0.0), m_armSubsystem));
      m_shooterSubsystem.setDefaultCommand(new TestShooter(m_shooterSubsystem, () -> -m_operatorController.getRightTriggerAxis(), () -> -m_operatorController.getLeftTriggerAxis()));

      m_operatorController.a()
          .whileTrue(Commands.run(() -> m_armSubsystem.setArmSpeed(0.1), m_armSubsystem));

      m_operatorController.b()
          .whileTrue(Commands.run(() -> m_armSubsystem.setArmSpeed(-0.1), m_armSubsystem));

    } else {
      
      // Set Default Command for Intake
      m_intakeSubsystem.setDefaultCommand(
        new MoveIntake(m_intakeSubsystem, () -> -m_operatorController.getRightY()));
        
      // Set Default Command for Uptake
      m_uptakeSubsystem.setDefaultCommand(
        new MoveUptake(m_uptakeSubsystem, () -> -m_operatorController.getLeftY()));
          
      // Set Default Command for Arm
      // m_armSubsystem.setDefaultCommand(Commands.run(() -> m_armSubsystem.stopArm(), m_armSubsystem));
      m_armSubsystem.setDefaultCommand(new HoldArmToPosition(m_armSubsystem));

      m_operatorController.povDown().whileTrue(
        Commands.run(() -> m_armSubsystem.setArmSpeed(-0.5), m_armSubsystem));

      m_operatorController.povUp().whileTrue(
        Commands.run(() -> m_armSubsystem.setArmSpeed(0.5), m_armSubsystem));

      // Set Default Command for Shooter
      m_shooterSubsystem.setDefaultCommand(
        new TestShooter(
            m_shooterSubsystem,
            () -> -m_operatorController.getRightTriggerAxis(),
            () -> -m_operatorController.getLeftTriggerAxis()));

      m_testCommandJoystick.button(JoystickConstants.BTN_Y)
          .whileTrue(new TuneUptakeAmpShot(
            m_uptakeSubsystem, 
            () -> m_testController.getRawButtonPressed(JoystickConstants.BUMPER_LEFT),
            () -> m_testController.getRawButtonPressed(JoystickConstants.BUMPER_RIGHT), 
            () -> m_testController.getRawButtonPressed(JoystickConstants.BTN_A), 
            () -> m_testController.getRawButtonPressed(JoystickConstants.BTN_B)
          ));

      // @TODOs
      // Operator
      // @TODO Tune handoff setpoint for the shooter arm
      // @TODO Tune amp setpoint for the shooter arm

      // Unjam //
      m_operatorButtonBoxController.button(CustomButtonBoxConstants.BTN_1)
          .whileTrue(new MoveIntake(m_intakeSubsystem, () -> 0.5));

      // Intake using the button box //
      m_operatorButtonBoxController.button(CustomButtonBoxConstants.BTN_2)
          .whileTrue(
              new MoveIntakeUntilNoteDetected(m_intakeSubsystem, () -> -0.8)
              .andThen(new MoveIntake(m_intakeSubsystem, () -> 0.1).withTimeout(0.25))
              .andThen(Commands.runOnce(() -> {
                m_ledSubsystem.set(LEDConstants.ORANGE);
                noteIsInIntakeEntry.setBoolean(true);
              }))
          );

      // Score Note to amp from intake using the button box //
      m_operatorButtonBoxController.button(CustomButtonBoxConstants.BTN_3)
          .onTrue(
              new MoveArmToPos(m_armSubsystem, ArmConstants.SHOOTER_GOAL)
              .andThen(new WaitCommand(0.5))
              .andThen(Commands.runOnce(() -> m_shooterSubsystem.setIdleMode(CANSparkMax.IdleMode.kBrake)))
              .andThen(
                  new MoveIntakeUptakeUntilNoteDetected(
                      m_intakeSubsystem, 
                      m_uptakeSubsystem, 
                      () -> - 0.6,
                      () -> -0.275)
                  .deadlineWith(new MoveShooter(m_shooterSubsystem, () -> -0.35))
              )
              .andThen(new MoveShooter(m_shooterSubsystem, () -> 0.0).withTimeout(0.5))
              .andThen(new MoveArmToPos(m_armSubsystem, ArmConstants.AMP_GOAL))
          );

      m_operatorButtonBoxController.button(CustomButtonBoxConstants.BTN_5)
      .onTrue(
          new MoveShooter(m_shooterSubsystem, () -> -0.5).withTimeout(0.5)
              .andThen(Commands.runOnce(() -> m_shooterSubsystem.setIdleMode(CANSparkMax.IdleMode.kCoast)))
              .andThen(new MoveArmToPos(m_armSubsystem, 0.4))
              .andThen(
                Commands.runOnce(() -> m_ledSubsystem.set(LEDConstants.GREEN))
              )
              .andThen(new MoveShooter(m_shooterSubsystem, () -> -0.0).withTimeout(0.1))
              .andThen(new MoveArmToPos(m_armSubsystem, ArmConstants.INITIAL_GOAL))
              .andThen(Commands.runOnce(() -> {
                m_ledSubsystem.set(LEDConstants.BLACK);
                noteIsInIntakeEntry.setBoolean(false);
              }))

      );

      // Ferry Note using the button box //
      m_operatorButtonBoxController.button(CustomButtonBoxConstants.BTN_4)
          .whileTrue(
            new MoveArmToPos(m_armSubsystem, ArmConstants.SHOOTER_GOAL)
            .andThen(
              new HoldArmToPosition(m_armSubsystem)
              .alongWith(
                new MoveShooter(m_shooterSubsystem, () -> -0.8)
              )
            )              
          ).whileFalse(
            // Lock the robot at 10 degrees heading before shooting
            new MoveIntake(m_intakeSubsystem, () -> -0.6)
            .alongWith(new MoveUptake(m_uptakeSubsystem, () -> -0.6)).withTimeout(1.0))
            .andThen(new MoveShooter(m_shooterSubsystem, () -> 0.0).withTimeout(0.5))
            .andThen(Commands.runOnce(() -> {
              m_ledSubsystem.set(LEDConstants.BLACK);
              noteIsInIntakeEntry.setBoolean(false);
            }))
          );
      

      m_operatorButtonBoxController.button(CustomButtonBoxConstants.BTN_7)
        .onTrue(
          new MoveIntakeUntilNoteDetected(m_intakeSubsystem, () -> -0.7)
          .andThen(new MoveIntake(m_intakeSubsystem, () -> 0.4).withTimeout(0.2)
          .raceWith(
              new MoveUptake(m_uptakeSubsystem, () -> 0.4)
          )
          .andThen(Commands.runOnce(() -> {
                    m_intakeSubsystem.set(0);
                    m_uptakeSubsystem.set(0);
                  }))
          .andThen(
            new MoveUptake(m_uptakeSubsystem, () -> -1.0).withTimeout(1.0))
            .andThen(new MoveIntake(m_intakeSubsystem, () -> -0.5)))
        );
        
      

      // Prime Uptake Shot //
      m_operatorButtonBoxController.button(CustomButtonBoxConstants.BTN_11)
        .whileTrue(
            new MoveArmToPos(m_armSubsystem, ArmConstants.INITIAL_GOAL)
            .andThen(new MoveIntake(m_intakeSubsystem, () -> 0.2).withTimeout(0.1))
            .andThen(Commands.runOnce(() -> m_intakeSubsystem.set(0.0)))
            .andThen(new MoveUptake(m_uptakeSubsystem, () -> -1.0))
        ).whileFalse(
          new MoveUptake(m_uptakeSubsystem, () -> -1.0)
          .alongWith(new MoveIntake(m_intakeSubsystem, () -> -1.0)).withTimeout(1.0)
          .andThen(Commands.runOnce(() -> {
            m_ledSubsystem.set(LEDConstants.BLACK);
            noteIsInIntakeEntry.setBoolean(false);
          }))
        );

      // Party Mode - AKA Trick Shot //
      m_operatorButtonBoxController.button(CustomButtonBoxConstants.BTN_12)
        .whileTrue(
            new MoveArmToPos(m_armSubsystem, ArmConstants.SHOOTER_GOAL)
            .andThen(
              new HoldArmToPosition(m_armSubsystem)
              .alongWith(
                new MoveShooter(m_shooterSubsystem, () -> 0.8)
              )
            )              
          ).whileFalse(
            new MoveIntake(m_intakeSubsystem, () -> -0.6)
            .alongWith(new MoveUptake(m_uptakeSubsystem, () -> -0.6)).withTimeout(1.0)
            .andThen(Commands.runOnce(() -> m_shooterSubsystem.set(0.0)))
            .andThen(Commands.runOnce(() -> {
                m_ledSubsystem.set(LEDConstants.BLACK);
                noteIsInIntakeEntry.setBoolean(false);
              }))
          );

      m_operatorButtonBoxController.button(CustomButtonBoxConstants.BTN_7)
          .whileTrue(
              new MoveIntake(m_intakeSubsystem, () -> 0.5)
          );


      // // RBump. MoveIntakeUntilNoteDetected
      // //   - Note travel distance depends on intake power
      // //   - apply a 0.1 second down for both intake and uptake at low percentage (20%) (if needed)
      // m_operatorController.rightBumper()
      //     .whileTrue(
      //         new MoveIntakeUntilNoteDetected(m_intakeSubsystem, () -> -0.65)
      //         .andThen(
      //             new MoveIntake(m_intakeSubsystem, () -> 0.65).withTimeout(0.25)
      //         )
      //     );

      // m_operatorController.leftBumper()
      // .whileTrue(
      //     new MoveIntakeUntilNoteDetected(m_intakeSubsystem, () -> -0.7)
      //     .andThen(new MoveIntake(m_intakeSubsystem, () -> 0.45).withTimeout(0.25))
      //     .andThen(Commands.runOnce(() -> m_ledSubsystem.set(LEDConstants.ORANGE)))
      // );

      // // A. PrimeUptakeShot
      // //   - Spin Uptake to (x%) power
      // m_operatorController.a().whileTrue(new MoveUptake(m_uptakeSubsystem, () -> -1.0));

      // // X. MoveIntake(100%) (Shoot using uptake)
      // //   - Move intake at 100%
      // m_operatorController.x().whileTrue(new MoveIntake(m_intakeSubsystem, () -> -1.0));

      // // Y. MoveUptake(50%) (Shoot using shooter)
      // m_operatorController.y().whileTrue(new MoveUptake(m_uptakeSubsystem, () -> -0.43));

      // // // Driver


      // Intake to Uptake - User should hold the button //

      // Intake Amp Sequence - User should hold the button //

      // Prepare to Shoot Speaker - User should hold the button //

      // Score Speaker - User should hold the button //

      // Score Amp - User should hold the button //
    }

    // Use the "A" button to reset the Gyro orientation //
    m_driverController.a().onTrue(Commands.runOnce(() -> m_swerveDriveSubsystem.zeroGyro()));

    // Use the "B" button to x-lock the wheels //
    m_driverController.b().onTrue(Commands.runOnce(() -> m_swerveDriveSubsystem.lock()));

    // // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    //#endregion

    //#region Joystick Bindings
    
    // // // Set default teleop command to drive //
    // // m_swerveDriveSubsystem.setDefaultCommand(
    // //   m_swerveDriveSubsystem.drive(
    // //     () -> -m_driverController.getRawAxis(JoystickConstants.STICK_LEFT_Y),   // Translation
    // //     () -> -m_driverController.getRawAxis(JoystickConstants.STICK_LEFT_X),   // Strafe
    // //     () -> -m_driverController.getRawAxis(JoystickConstants.STICK_RIGHT_X)   // Rotation
    // //   )
    // // );

    // // m_swerveDriveSubsystem.setDefaultCommand(
    // //   m_swerveDriveSubsystem.driveHeading(
    // //     () -> -m_driverController.getRawAxis(JoystickConstants.STICK_LEFT_Y),   // Translation
    // //     () -> -m_driverController.getRawAxis(JoystickConstants.STICK_LEFT_X),   // Strafe
    // //     () -> -m_driverController.getRawAxis(JoystickConstants.STICK_RIGHT_Y),  // X component of angle
    // //     () -> -m_driverController.getRawAxis(JoystickConstants.STICK_RIGHT_Y)   // Y component of angle
    // //   )
    // // );

    // if (GeneralConstants.CURRENT_MODE == RobotMode.TEST){
    //   // m_climberSubsystem.setDefaultCommand(new TestClimber(m_climberSubsystem));
    //   m_intakeSubsystem.setDefaultCommand(
    //       new TestIntake(
    //           m_intakeSubsystem,
    //           () -> -m_operatorController.getRawAxis(JoystickConstants.STICK_RIGHT_Y)
    //       )
    //   );

    //   m_uptakeSubsystem.setDefaultCommand(
    //       new TestUptake(
    //           m_uptakeSubsystem,
    //           () -> -m_operatorController.getRawAxis(JoystickConstants.STICK_LEFT_Y)
    //       )
    //   );

    //   m_armSubsystem.setDefaultCommand(
    //       Commands.run(() -> m_armSubsystem.setArmSpeed(0.0), m_armSubsystem));
    //   m_shooterSubsystem.setDefaultCommand(
    //       new TestShooter(
    //           m_shooterSubsystem,
    //           () -> -m_operatorController.getRawAxis(JoystickConstants.TRIGGER_RIGHT),
    //           () -> -m_operatorController.getRawAxis(JoystickConstants.TRIGGER_LEFT)
    //       )
    //   );

    //   m_operatorController.button(JoystickConstants.BTN_A)
    //       .whileTrue(Commands.run(() -> m_armSubsystem.setArmSpeed(0.1), m_armSubsystem));

    //   m_operatorController.button(JoystickConstants.BTN_B)
    //       .whileTrue(Commands.run(() -> m_armSubsystem.setArmSpeed(-0.1), m_armSubsystem));
    // } else {
    //   // m_climberSubsystem.setDefaultCommand(new TestClimber(m_climberSubsystem));
    //   m_intakeSubsystem.setDefaultCommand(Commands.run(() -> m_intakeSubsystem.stopIntake(), m_intakeSubsystem));
    //   m_uptakeSubsystem.setDefaultCommand(Commands.run(() -> m_uptakeSubsystem.stopUptake(), m_uptakeSubsystem));
    //   m_armSubsystem.setDefaultCommand(Commands.run(() -> m_armSubsystem.stopArm(), m_armSubsystem));
    //   m_shooterSubsystem.setDefaultCommand(Commands.run(() -> m_shooterSubsystem.stopShooter(), m_shooterSubsystem));

    //   m_operatorController.button(JoystickConstants.BTN_A)
    //       .whileTrue(Commands.run(() -> m_armSubsystem.setArmSpeed(0.1), m_armSubsystem));

    //   m_operatorController.button(JoystickConstants.BTN_B)
    //       .whileTrue(Commands.run(() -> m_armSubsystem.setArmSpeed(-0.1), m_armSubsystem));

    //   // Intake to Uptake - User should hold the button //

    //   // Intake Amp Sequence - User should hold the button //

    //   // Prepare to Shoot Speaker - User should hold the button //

    //   // Score Speaker - User should hold the button //

    //   // Score Amp - User should hold the button //
    // }

    // // Use the "A" button to reset the Gyro orientation //
    // m_driverController.button(JoystickConstants.BTN_A)
    //     .onTrue(Commands.runOnce(() -> m_swerveDriveSubsystem.zeroGyro()));

    //#endregion
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // return null;
  }
}
