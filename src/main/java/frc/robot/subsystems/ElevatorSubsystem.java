package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants;



public class ElevatorSubsystem extends SubsystemBase {
  private final WPI_VictorSPX m_elevatorMotor1 = new WPI_VictorSPX(ElevatorConstants.kElevatorMotor1Id);
  private final WPI_VictorSPX m_elevatorMotor2 = new WPI_VictorSPX(ElevatorConstants.kElevatorMotor2Id);

  // private final PIDController elevatorController = new PIDController(
  //     ElevatorConstants.kElevatorMotorKp,
  //     ElevatorConstants.kElevatorMotorKi,
  //     ElevatorConstants.kElevatorMotorKd
  // );
  // private final SimpleMotorFeedforward elevatorFeedforward = new SimpleMotorFeedforward(
  //     ElevatorConstants.kElevatorMotorKs,
  //     ElevatorConstants.kElevatorMotorKv
  // );
  private final DutyCycleEncoder m_absoluteEncoder = new DutyCycleEncoder(ElevatorConstants.kEncoderId);

  private double m_maxHeight = 100;
  private double m_minHeight = 0;

  private double m_staticVoltage = 2;
  private double m_raisingVoltage = 6;
  private double m_loweringVoltage = -10;
  
  public double m_lastVoltage = 0;

  private boolean isElevatorFullyOpened = false;
  private boolean isElevatorFullyClosed = true;

  public ElevatorSubsystem() {
    // elevatorMotor1.configVoltageCompSaturation(Constants.nominalVoltage);
    // elevatorMotor1.enableVoltageCompensation(true);

    // elevatorMotor2.configVoltageCompSaturation(Constants.nominalVoltage);
    // elevatorMotor2.enableVoltageCompensation(true);
    m_absoluteEncoder.reset();
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Elevator Encoder Raw Reading", m_absoluteEncoder.getDistance());
    SmartDashboard.putNumber("Elevator Encoder Reading", getHeight());
  }

  public double getHeight() {
    return m_absoluteEncoder.getDistance();
  }

  private void setVoltage() {
    if (getHeight() > m_maxHeight && m_lastVoltage == m_raisingVoltage) {
      m_lastVoltage = m_staticVoltage;
    }

    if (getHeight() < m_minHeight && m_lastVoltage == m_loweringVoltage) {
      m_lastVoltage = m_staticVoltage;
    }
    if (getHeight() < m_minHeight && m_lastVoltage == m_loweringVoltage) {
      m_lastVoltage = m_staticVoltage;
    }

    m_elevatorMotor1.setVoltage(m_lastVoltage);
    m_elevatorMotor2.setVoltage(m_lastVoltage);
  }

  private void setVoltage(double voltage) {
    m_lastVoltage = voltage;
    setVoltage();
  }

  private void openElevatorOnce() {
    setVoltage(m_raisingVoltage);
  }

  private void closeElevatorOnce() {
    setVoltage(m_loweringVoltage);
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
    // setVoltage();
  }
}
