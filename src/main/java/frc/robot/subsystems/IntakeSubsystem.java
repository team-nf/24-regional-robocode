package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Encoder;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final WPI_VictorSPX intakeMotor = new WPI_VictorSPX(IntakeConstants.kIntakeMotorId);;
  private final PIDController intakeController = new PIDController(
    IntakeConstants.kIntakeMotorKp,
    IntakeConstants.kIntakeMotorKi,
    IntakeConstants.kIntakeMotorKd
  );
  private final SimpleMotorFeedforward intakeFeedForward = new SimpleMotorFeedforward(
    IntakeConstants.kIntakeMotorKs,
    IntakeConstants.kIntakeMotorKv
  );
  private final Encoder intakeEncoder = new Encoder(
    IntakeConstants.kIntakeEncoderA,
    IntakeConstants.kIntakeEncoderB
  );
  private double currentTargetVel;


  public IntakeSubsystem() {
    intakeMotor.configVoltageCompSaturation(Constants.nominalVoltage);
    intakeMotor.enableVoltageCompensation(true);
    currentTargetVel = 0;
  }

  private void setVelOnce() {
    double currentVel = intakeEncoder.getRate();
    double pidValue = intakeController.calculate(currentVel, currentTargetVel);
    double feedforwardValue = intakeFeedForward.calculate(currentTargetVel);

    intakeMotor.set(pidValue + feedforwardValue);
  }

  private void setVelOnce(double targetVel) {
    currentTargetVel = targetVel;
    setVelOnce();
  }

  public Command runIntakeCommand() {
    return runOnce(() -> {
        setVelOnce(IntakeConstants.kIntakeTargetVel);
    });
  }

  public Command stopIntakeCommand() {
    return runOnce(() -> {
        setVelOnce(0);
    });
  }

  /**
   * WPILib Pid loopu sürekli dönmeli
   */
  @Override
  public void periodic() {
    setVelOnce();
  }
}
