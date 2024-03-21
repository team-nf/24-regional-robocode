package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

// import edu.wpi.first.wpilibj2.command.FunctionalCommand;
// import java.util.function.BooleanSupplier;
// import java.util.function.Consumer;


public class IntakeSubsystem extends SubsystemBase {
  private final WPI_VictorSPX intakeMotor = new WPI_VictorSPX(IntakeConstants.kIntakeMotorId);
  private double m_lastVoltage = 0;

  public IntakeSubsystem() {
    intakeMotor.setInverted(false);
  }

  public void setVoltage() {
    intakeMotor.setVoltage(m_lastVoltage);
  }

  public void setVoltage(double voltage) {
    m_lastVoltage = voltage;
    setVoltage();
  }

  public void runIntake() {
    setVoltage(IntakeConstants.kIntakeRunVoltage);
  }

  public void runIntakeReverse() {
    setVoltage(-IntakeConstants.kIntakeReverseVoltage);
  }

  public void stopIntake() {
    setVoltage(0);
  }

  public Command runIntakeCommand() { return runOnce(() -> runIntake()); }
  public Command runIntakeReverseCommand() { return runOnce(() -> runIntakeReverse()); }
  public Command stopIntakeCommand() { return runOnce(() -> stopIntake()); }

  @Override
  public void periodic() {
    setVoltage();
  }
}
