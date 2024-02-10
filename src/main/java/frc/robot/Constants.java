// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // public static final Mode currentMode = Mode.REAL;

  // public static enum Mode {
  //   /** Running on a real robot. */
  //   REAL,

  //   /** Running a physics simulator. */
  //   SIM,

  //   /** Replaying from a log file. */
  //   REPLAY
  // }

  public static final Mode CURRENT_MODE = Mode.COMP;

  public static enum Mode {
    /** Running on test mode */
    TEST,
    /** Running on competition mode */
    COMP
  }

    /** General robot constants */
  public static final class GeneralConstants {
    // Enable or disable competition mode
    public static final boolean tuningMode = true;

    // Joystick axis deadband for the swerve drive
    public static final double swerveDeadband = 0.1;

    public static final double voltageComp = 10.0;

    // Hold time on motor brakes when disabled
    public static final double wheelLockTime = 10;

    public static final double robotMass = (148 - 20.3) * 0.453592;
    public static final double chassisMass = robotMass;
    public static final Translation3d chassisCG = new Translation3d(0, 0, Units.inchesToMeters(8));
    public static final double loopTime = 0.13;
  }

  /** Intake Subsystem Constants */
  public static class IntakeConstants {
    public static final int MOTOR_ID = 0;
    
    public static final double NOMINAL_VOLTAGE = 8.0;
    public static final int STALL_CURRENT_LIMIT = 20;
    public static final int FREE_CURRENT_LIMIT = 10;
    public static final int RPM_LIMIT = 3000;
    public static final double SECONDARY_CURRENT_LIMIT = 40.0;
    public static final double RAMP_RATE_IN_SEC = 0.25; // Ramp rate in seconds
    
    public static final int BEAM_BREAK_DIGITAL_CHANNEL = 0;
  }

  /** Uptake Subsystem Constants */
  public static class UptakeConstants {
    public static final int LEFT_MOTOR_ID = 1;
    public static final int RIGHT_MOTOR_ID = 2;
    
    public static final double NOMINAL_VOLTAGE = 8.0;
    public static final int STALL_CURRENT_LIMIT = 20;
    public static final int FREE_CURRENT_LIMIT = 10;
    public static final double SECONDARY_CURRENT_LIMIT = 40.0;
    public static final double RAMP_RATE_IN_SEC = 0.25; // Ramp rate in seconds
    
    public static final int TOP_BEAM_BREAK_DIGITAL_CHANNEL = 1;
    public static final int BOTTOM_BEAM_BREAK_DIGITAL_CHANNEL = 2;
  }

  /** ShooterAndAmp Subsystem Constants */
  public static class ShooterConstants {
    public static final int TOP_MOTOR_ID = 3;
    public static final int BOTTOM_MOTOR_ID = 4;

    public static final double NOMINAL_VOLTAGE = 10.0;
    public static final int STALL_CURRENT_LIMIT = 40;
    public static final int FREE_CURRENT_LIMIT = 20;
    public static final double SECONDARY_CURRENT_LIMIT = 60.0;
    public static final double RAMP_RATE_IN_SEC = 0.25; // Ramp rate in seconds

    public static final int BEAM_BREAK_DIGITAL_CHANNEL = 3;

    public static final double P = 0.0;
    public static final double I = 0.0;
    public static final double D = 0.0;
    public static final double F = 0.0;
    public static final double IZ = 0.0;
    public static final double MIN_OUTPUT = -0.2;
    public static final double MAX_OUTPUT = 0.8;

    /* Desired and max RPM for the shooter (to be tuned later) */
    public static final double DESIRED_SHOOTER_RPM = 4000;
    public static final double MAX_RPM = 5000;
  }

  public static class AmpConstants {
    public static final int LEFT_MOTOR_ID = 5;
    public static final int RIGHT_MOTOR_ID = 6;

    public static final double NOMINAL_VOLTAGE = 10.0;
    public static final int STALL_CURRENT_LIMIT = 40;
    public static final double SECONDARY_CURRENT_LIMIT = 60.0;

    public static final double P = 0.0;
    public static final double I = 0.0;
    public static final double D = 0.0;
    public static final double F = 0.0;
    public static final double IZ = 0.0;

    /* Desired absolute encoder setpoint for moving shooter and amp mechanism (to be tuned later using absolute encoder) */
    public static final double INITIAL_SETPOINT = 0.0;
    public static final double SHOOTER_SETPOINT = 0.0;
    public static final double AMP_SETPOINT = 0.0;
  }

  /** Climber Subsystem Constants */
  public static class ClimberConstants {
    public static final int LEFT_MOTOR_ID = 7;
    public static final int RIGHT_MOTOR_ID = 8;

    public static final double NOMINAL_VOLTAGE = 10.0;
    public static final int STALL_CURRENT_LIMIT = 40;
    public static final double SECONDARY_CURRENT_LIMIT = 60.0;

    public static final float STARTING_LIMIT = 0.0f;
    public static final float ENDING_LIMIT = 100.0f;
  }

  /** Limelight Subsystem Constants */
  /** PhotonVision Subsystem Constants */
  
  public static class SwerveConstants {
    // General constants for swerve drive //
    public static final double ANGLE_GEAR_RATIO = 12.8;
    public static final double DRIVE_GEAR_RATIO = 8.14;
    public static final double PULSE_PER_ROTATION = 1;
    public static final double WHEEL_DIAMETER_INCHES = 4.0;
    public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES);
    public static final double MAX_SPEED_FEET_PER_SECOND = 12.5; // 12.5 feet per second
    public static final double MAX_SPEED_METERS_PER_SECOND = Units.feetToMeters(MAX_SPEED_FEET_PER_SECOND); // 12.5 feet per second
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kOperatorButtonControllerPort = 2;
  }

  public static class CustomButtonBoxConstants {
    public static final int BTN_1 = 1;
    public static final int BTN_2 = 2;
    public static final int BTN_3 = 3;
    public static final int BTN_4 = 4;
    public static final int BTN_5 = 5;
    public static final int BTN_6 = 6;
    public static final int BTN_7 = 7;
    public static final int BTN_8 = 8;
    public static final int BTN_9 = 9;
    public static final int BTN_10 = 10;
    public static final int BTN_11 = 11;
    public static final int BTN_12 = 12;        
  }

  public static class JoystickConstants {
    // Joystick Analog Axis/Stick //
    public static final int STICK_LEFT_X = 0;
    public static final int STICK_LEFT_Y = 1;
    public static final int TRIGGER_LEFT = 2;
    public static final int TRIGGER_RIGHT = 3;
    public static final int STICK_RIGHT_X = 4;
    public static final int STICK_RIGHT_Y = 5;

    // Joystick Buttons //
    public static final int BTN_A = 1;
    public static final int BTN_B = 2;
    public static final int BTN_X = 3;
    public static final int BTN_Y = 4;
    public static final int BUMPER_LEFT = 5;
    public static final int BUMPER_RIGHT = 6;
    public static final int BTN_BACK = 7;
    public static final int BTN_START = 8;
    public static final int BTN_STICK_LEFT = 9;
    public static final int BTN_STICK_RIGHT = 10;
  }

  public static class Extreme3DProConstants {
    // Extreme 3D Pro Analog Axis/Stick //
    public static final int X_AXIS = 0;
    public static final int Y_AXIS = 1;
    public static final int ROTATE = 2;
    public static final int SLIDER = 3;

    // Extreme 3D Pro Buttons //
    public static final int BTN_TRIGGER = 1;
    public static final int BTN_THUMB = 2;
    public static final int BTN_BOT_LEFT = 3;
    public static final int BTN_BOT_RIGHT = 4;
    public static final int BTN_TOP_LEFT = 5;
    public static final int BTN_TOP_RIGHT = 6;
    public static final int BTN_7 = 7;
    public static final int BTN_8 = 8;
    public static final int BTN_9 = 9;
    public static final int BTN_10 = 10;
    public static final int BTN_11 = 11;
    public static final int BTN_12 = 12;
  }
}
