package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final WPI_VictorSPX intakeMotor;

  public IntakeSubsystem() {
    intakeMotor = new WPI_VictorSPX(IntakeConstants.kIntakeMotorId);

    // Motor configuration
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.setInverted(true); //?
  }

  public void intake(double speed) {
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public Command intakeCommand(DoubleSupplier speedSupplier) {
    return runOnce(() -> intake(speedSupplier.getAsDouble())); 
  }

  public Command pullInCommand() {
    return intakeCommand(() -> IntakeConstants.kDefaultIntakeSpeed);
  }

  public Command stopIntakeCommand() {
    return intakeCommand(() -> 0);
  }

  public Command holdInCommand() {
    return intakeCommand(() -> IntakeConstants.kDefaultHoldSpeed);
  }

  public Command slowOutCommand() {
    return intakeCommand(() -> IntakeConstants.kDefaultSlowOutSpeed);
  }
}
