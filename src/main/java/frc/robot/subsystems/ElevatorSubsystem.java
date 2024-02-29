package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants;



public class ElevatorSubsystem extends SubsystemBase {
  private final CANSparkMax elevatorMotor1 = new CANSparkMax(ElevatorConstants.kElevatorMotor1Id, MotorType.kBrushless);
  private final CANSparkMax elevatorMotor2 = new CANSparkMax(ElevatorConstants.kElevatorMotor2Id, MotorType.kBrushless);

  private final PIDController elevatorController = new PIDController(
      ElevatorConstants.kElevatorMotorKp,
      ElevatorConstants.kElevatorMotorKi,
      ElevatorConstants.kElevatorMotorKd
  );
  private final SimpleMotorFeedforward elevatorFeedforward = new SimpleMotorFeedforward(
      ElevatorConstants.kElevatorMotorKs,
      ElevatorConstants.kElevatorMotorKv
  );
  private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(ElevatorConstants.kEncoderId);

  private double currentTargetHeight;


  public ElevatorSubsystem() {
    elevatorMotor1.enableVoltageCompensation(Constants.nominalVoltage);
    elevatorMotor1.setInverted(true);

    elevatorMotor2.enableVoltageCompensation(Constants.nominalVoltage);
    elevatorMotor2.setInverted(true);
  }

  private void setHeightOnce() {
    double currentPosition = absoluteEncoder.getAbsolutePosition();
    double pidValue = elevatorController.calculate(currentPosition, currentTargetHeight);
    double feedforwardValue = elevatorFeedforward.calculate(currentTargetHeight, ElevatorConstants.kMoveVel);

    elevatorMotor1.set(pidValue + feedforwardValue);
    elevatorMotor2.set(pidValue + feedforwardValue);
  }

  private void setHeightOnce(double height) {
    this.currentTargetHeight = height;
    setHeightOnce();
  }

  private Command setAndWaitPosition(double targetHeight, double acceptableHeightError) {
    Consumer<Boolean> onEnd = wasInterrupted -> {
        System.out.println("setAndWaitPosition ended");
    };

    BooleanSupplier hasReachedVelocity = () ->
        Math.abs(absoluteEncoder.getAbsolutePosition() - targetHeight) <= acceptableHeightError;

    return new FunctionalCommand(
        () -> {},
        () -> setHeightOnce(targetHeight),
        onEnd,
        hasReachedVelocity
    );
  }

  public Command closeElevatorOnceCommand() {
    return runOnce(() -> {
      setHeightOnce(ElevatorConstants.kElevatorMinHeight);
    });
  }

  public Command closeElevatorCommand() {
    return setAndWaitPosition(currentTargetHeight, currentTargetHeight);
  }

  public Command openElevatorOnceCommand() {
    return runOnce(() -> {
      setHeightOnce(ElevatorConstants.kElevatorMaxHeight);
    });
  }

  public Command openElevatorCommand() {
    return setAndWaitPosition(currentTargetHeight, currentTargetHeight);
  }

  public double getHeight() {
    return absoluteEncoder.getAbsolutePosition();
  }

  public boolean isElevatorFulyOpen() {
    return absoluteEncoder.getAbsolutePosition() == ElevatorConstants.kElevatorMaxHeight;
  }

  public boolean isElevatorFullyClosed() {
    return absoluteEncoder.getAbsolutePosition() == ElevatorConstants.kElevatorMinHeight;
  }

  @Override
  public void periodic() {
    setHeightOnce();
  }
}
