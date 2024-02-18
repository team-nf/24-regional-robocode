// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class IntakeConstants {
    public static final int kIntakeMotorId = 1; 
    public static final int kUpperMotorId = 2;
    public static final double kDefaultIntakeSpeed = -0.5;
    public static final double kDefaultHoldSpeed = -0.2;
    public static final double kDefaultSlowOutSpeed = 0.45;
    public static final double kMaxIntakeSpeed = 1.0;
    public static final double kMinIntakeSpeed = -0.1;
    public static final double kWaitTime = 3.0;
  }

  public static final class ShooterConstants {
    public static double kFlywheelOnTargetTolerance = 100.0;
    public static double kFlywheelGoodBallRpmSetpoint = 6200.0;
    public static double kFlywheelBadBallRpmSetpoint = kFlywheelGoodBallRpmSetpoint;
    public static double kFlywheelKp = 0.12;
    public static double kFlywheelKi = 0.0;
    public static double kFlywheelKd = 0.5;
    public static double kFlywheelKf = 0.014;
    public static int kFlywheelIZone = (int) (1023.0 / kFlywheelKp);
    public static double kFlywheelRampRate = 0;
    public static int kFlywheelAllowableError = 0;
    public static final int kShooterMotorId = 1;
    
  }
}
