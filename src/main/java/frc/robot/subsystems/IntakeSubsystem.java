package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.IntakeConstants;




public class IntakeSubsystem extends SubsystemBase {
  private final WPI_VictorSPX intakeMotor;
  private final WPI_VictorSPX upperMotor;

  public IntakeSubsystem() {
    intakeMotor = new WPI_VictorSPX(IntakeConstants.kIntakeMotorId);
    upperMotor = new WPI_VictorSPX(IntakeConstants.kIntakeMotorId);

    // Motor configuration
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.setInverted(true);

    // Motor configuration
    upperMotor.setNeutralMode(NeutralMode.Brake);
    upperMotor.setInverted(true);
  }

  public void runIntake(double speed) {
    intakeMotor.setVoltage(2);
  }

  public void runUpper(double speed) {
    upperMotor.set(ControlMode.PercentOutput, speed);
  }

  public Command getIntakeRunCommand(DoubleSupplier speedSupplier) {
    return runOnce(() -> runIntake(speedSupplier.getAsDouble())); 
  }

  public Command getUpperRunCommand(DoubleSupplier speedSupplier) {
    return runOnce(() -> runUpper(speedSupplier.getAsDouble())); 
  }

  public Command getPullInCommand() {
    return new SequentialCommandGroup(
      getIntakeRunCommand(() -> IntakeConstants.kDefaultIntakeSpeed),
      new WaitCommand(3.),
      stopIntakeCommand()
    );
  }

  public Command stopIntakeCommand() {
    return getIntakeRunCommand(() -> 0);
  }

  public Command holdInCommand() {
    return getIntakeRunCommand(() -> IntakeConstants.kDefaultHoldSpeed);
  }

  public Command getSlowOutCommand() {
    return getIntakeRunCommand(() -> IntakeConstants.kDefaultSlowOutSpeed);
  }
}
