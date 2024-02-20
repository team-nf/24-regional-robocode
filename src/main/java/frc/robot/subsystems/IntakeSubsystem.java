package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

// import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
  private final WPI_VictorSPX intakeMotor;

  public IntakeSubsystem() {
    intakeMotor = new WPI_VictorSPX(IntakeConstants.kIntakeMotorId);

    // Motor configuration
    intakeMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.setInverted(true);
  }

  public void runIntake(double voltage) {
    intakeMotor.setVoltage(voltage);
  }

  public void runIntake() {
    runIntake(IntakeConstants.kMinVoltage);
  }

  public Command runIntakeCommand(DoubleSupplier voltageSupplier) {
    return run(() -> runIntake(voltageSupplier.getAsDouble()));
  }

  public Command runIntakeCommand() {
    return run(() -> runIntake());
  }

  public Command stopIntakeCommand() {
    return run(() -> runIntake(0));
  }
}
