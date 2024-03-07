// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants
  {

    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {
    public static final int CONTROLLER_PORT = 0;
    
    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;

    // Generic HID Axis numbers, Button numbers (Controller Mapping)
    
    // FIX
    public static final int BUTTON_A = 0;
    public static final int BUTTON_B = 1;
    public static final int BUTTON_X = 2;
    public static final int BUTTON_Y = 3;
    public static final int LEFT_BUMPER = 6;
    public static final int LEFT_TRIGGER = 7;
    public static final int RIGHT_BUMPER = 8;
    public static final int RIGHT_TRIGGER = 9;

    // YAGSL INHERITED BINDING BUTTONS
    public static int ZERO_GYRO_BUTTON;
    public static int FAKE_VISION_TRIGGER;
    public static int ACTION_TRIGGER;
    public static int LOCK_DRIVEBASE_TRIGGER;

    // FIX
    public static final int LEFT_X_AXIS = 0;
    public static final int LEFT_Y_AXIS = 1;
    public static final int RIGHT_X_AXIS = 2;
    public static final int RIGHT_Y_AXIS = 3;
    
  }

  public static final double nominalVoltage = 9.0;

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class IntakeConstants {
    public static final int kIntakeMotorId = 0;
    public static final int kIntakeEncoderA = 1;
    public static final int kIntakeEncoderB = 2;

    public static final int kMoveVel = 2;

    public static final int kIntakeMotorKp = 1;
    public static final int kIntakeMotorKi = 1;
    public static final int kIntakeMotorKd = 1;
    public static final int kIntakeMotorKs = 1;
    public static final int kIntakeMotorKv = 1;

    public static final int kIntakeTargetVel = 1;
  }

  public static final class ElevatorConstants {
    public static final double kElevatorMinHeight = 0;
    public static final double kElevatorMaxHeight = 100;

    public static final int kElevatorMotor1Id = 1;
    public static final int kElevatorMotor2Id = 2;
    public static final int kEncoderId = 2;

    public static final int kMoveVel = 2;

    public static final int kElevatorMotorKp = 1;
    public static final int kElevatorMotorKi = 1;
    public static final int kElevatorMotorKd = 1;
    public static final int kElevatorMotorKs = 1;
    public static final int kElevatorMotorKv = 1;

  }

  public static final class ShooterConstants {
    // public static double kFlywheelOnTargetTolerance = 100.0;
    // public static double kFlywheelGoodBallRpmSetpoint = 6200.0;
    // public static double kFlywheelBadBallRpmSetpoint = kFlywheelGoodBallRpmSetpoint;
    // public static double kFlywheelKp = 0.12;
    // public static double kFlywheelKi = 0.0;
    // public static double kFlywheelKd = 0.5;
    // public static double kFlywheelKf = 0.014;
    // public static int kFlywheelIZone = (int) (1023.0 / kFlywheelKp);
    // public static double kFlywheelRampRate = 0;
    // public static int kFlywheelAllowableError = 0;
    // public static final int kShooterMotorId = 1;

    /* Motor, encoder ve sensör idleri */
    public static int kThrowerMotorLowerId = 3;
    public static int kThrowerMotorUpperId = 4;
    public static int kFeederMotorId = 5;
    public static int kAngleMotorId = 6;
    public static int kAngleEncoderId = 0;
    public static int kObjectSensorPort = 7;

    /* Açı, hız ve voltaj bilgileri */
    public static int kThrowerVelocity;
    public static int kFeederVelocity;
    public static int kFeederReverseVelocity;
    public static int kClimbingAngle;
    public static int kFeedAngle;
    public static int kIntakeAngle;
    public static int kTrapThrowAngle;
    public static int kAngularVel = 1;

    public static double kSVolts = 1;
    public static double kVVoltSecondsPerRotation = 1;

    /* PID Sabitleri */
    public static int kUpperThrowerKp = 1;
    public static int kUpperThrowerKi = 1;
    public static int kUpperThrowerKd = 1;
    public static int kUpperThrowerKf = 1;

    public static int kLowerThrowerKp = 1;
    public static int kLowerThrowerKi = 1;
    public static int kLowerThrowerKd = 1;
    public static int kLowerThrowerKf = 1;

    public static int kAngleKp = 1;
    public static int kAngleKi = 1;
    public static int kAngleKd = 1;
    public static int kAngleKf = 1;
    public static int kAngleKg = 1;
    public static int kAngleKs = 1;
    public static int kAngleKv = 1;

    public static double kAngleInitialPos = 1;
    public static double kAngleToleranceRPS = 1;

    public static int kFeederKp = 1;
    public static int kFeederKi = 1;
    public static int kFeederKd = 1;
    public static int kFeederKf = 1;
  }
}
