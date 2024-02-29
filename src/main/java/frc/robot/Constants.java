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

 public static class ArmSimConstants {
    public static final int kMotorPort = 0;
    public static final int kEncoderAChannel = 0;
    public static final int kEncoderBChannel = 1;
    public static final int kJoystickPort = 0;

    // The P gain for the PID controller that drives this arm.
    public static final double kArmKp = 40.0;
    public static final double kArmKi = 0.0;

    // distance per pulse = (angle per revolution) / (pulses per revolution)
    // = (2 * PI rads) / (4096 pulses)
    public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

    // Simulation classes help us simulate what's going on, including gravity.
    public static final double m_armGravity = 60;
    public static final double m_arm_topMass = 10.0; // Kilograms
    public static final double m_arm_topLength = Units.inchesToMeters(38.5);
    public static final double m_arm_bottomMass = 4.0; // Kilograms
    public static final double m_arm_bottomLength = Units.inchesToMeters(27);

    public static final int m_arm_top_min_angle = -75;
    public static final int m_arm_top_max_angle = 260;
    public static final int m_arm_bottom_min_angle = 30;
    public static final int m_arm_bottom_max_angle = 150;

  }

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
 public static class setPoint {
    // SETPOINTS
  public static final int defaultBottomPosition = 90;
  public static final int defaultTopPosition = 260;

  public static final int positionOneBottom = 140;
  public static final int positionOneTop = 200;

  public static final int positionThreeBottom = 135;
  public static final int positionThreeTop = 160;

  public static final int positionFourBottom = 120;
  public static final int positionFourTop = 255;

  public static final int positionTwoBottom = 60;
  public static final int positionTwoTop = 195;
  }
  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
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
