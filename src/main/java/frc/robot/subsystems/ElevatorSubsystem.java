package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmSimConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants;



public class ElevatorSubsystem extends SubsystemBase {
 private final CANSparkMax elevatorMotor1 = new CANSparkMax(ElevatorConstants.kElevatorMotor1Id, MotorType.kBrushless);
 private final CANSparkMax elevatorMotor2 = new CANSparkMax(ElevatorConstants.kElevatorMotor2Id, MotorType.kBrushless);
private final DCMotor mArmFalcon = DCMotor.getFalcon500(2);
private final Encoder topEncoder = new Encoder(0, 2);
private final Encoder bottomEncoder = new Encoder(0, 2);



  private final EncoderSim m_topEncoderSim = new EncoderSim(topEncoder);
  private final EncoderSim m_bottomEncoderSim = new EncoderSim(bottomEncoder);

  private final Mechanism2d m_mech2d = new Mechanism2d(90, 90,new Color8Bit(Color.kWhite));

  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 55, 21.75);




  /**
   * Run the control loop to reach and maintain the setpoint from the preferences.
   */

  private final SingleJointedArmSim m_arm_topSim = new SingleJointedArmSim(
      mArmFalcon,
      ArmSimConstants.m_armGravity,
      SingleJointedArmSim.estimateMOI(ArmSimConstants.m_arm_topLength, ArmSimConstants.m_arm_topMass),
      ArmSimConstants.m_arm_topLength, // The length of the arm.
      Units.degreesToRadians(ArmSimConstants.m_arm_top_min_angle), // The minimum angle that the arm is capable of.
      Units.degreesToRadians(ArmSimConstants.m_arm_top_max_angle), // The maximum angle that the arm is capable of.
      false, // Whether gravity should be simulated or not.
      2, VecBuilder.fill(ArmSimConstants.kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
  );
    private final SingleJointedArmSim m_arm_bottomSim = new SingleJointedArmSim(
      mArmFalcon,
      ArmSimConstants.m_armGravity,
      SingleJointedArmSim.estimateMOI(ArmSimConstants.m_arm_topLength, ArmSimConstants.m_arm_topMass),
      ArmSimConstants.m_arm_topLength, // The length of the arm.
      Units.degreesToRadians(ArmSimConstants.m_arm_top_min_angle), // The minimum angle that the arm is capable of.
      Units.degreesToRadians(ArmSimConstants.m_arm_top_max_angle), // The maximum angle that the arm is capable of.
      false, // Whether gravity should be simulated or not.
      2, VecBuilder.fill(ArmSimConstants.kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
  );

 public final ProfiledPIDController topController = new ProfiledPIDController(ArmSimConstants.kArmKp, ArmSimConstants.kArmKi,
      0,
      new TrapezoidProfile.Constraints(2, 5));
  public final ProfiledPIDController bottomController = new ProfiledPIDController(ArmSimConstants.kArmKp,
      ArmSimConstants.kArmKi, 0,
      new TrapezoidProfile.Constraints(2, 5));

  private final MechanismLigament2d m_arm_bottom = m_armPivot.append(
      new MechanismLigament2d(
          "Arm Bottom",
          20,
          -90,
          10,
          new Color8Bit(Color.kDarkTurquoise)));
  public final MechanismLigament2d m_arm_tower = m_armPivot
      .append(new MechanismLigament2d("ArmTower", 18, -90, 10, new Color8Bit(Color.kDarkGray)));

  private final MechanismLigament2d m_arm_top = m_arm_bottom.append(
      new MechanismLigament2d(
          "Arm Top",
          28.5,
          Units.radiansToDegrees(m_arm_topSim.getAngleRads()),
          10,
          new Color8Bit(Color.kDarkSeaGreen)));


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
      topEncoder.setDistancePerPulse(ArmSimConstants.kArmEncoderDistPerPulse);
    bottomEncoder.setDistancePerPulse(ArmSimConstants.kArmEncoderDistPerPulse);
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

  public boolean isElevatorOpen() {
    return absoluteEncoder.getAbsolutePosition() > ElevatorConstants.kElevatorMinHeight;
  }


  public double pidoutputTop(int topSetpoint, int bottomSetpoint) {
    double pidOutputTop = topController.calculate(topEncoder.getDistance(),
        Units.degreesToRadians(topSetpoint - bottomSetpoint));
    return pidOutputTop;
  }

  public void setvoltage(int topSetpoint, int bottomSetpoint) {
    double pidOutputTop = topController.calculate(topEncoder.getDistance(),
        Units.degreesToRadians(topSetpoint - bottomSetpoint));
    elevatorMotor2.setVoltage(pidOutputTop);
    SmartDashboard.putNumber("Setpoint bottom (degrees)", bottomSetpoint);
    SmartDashboard.putNumber("Setpoint top (degrees)", topSetpoint);
    double pidOutputBottom = bottomController.calculate(bottomEncoder.getDistance(),
        Units.degreesToRadians(bottomSetpoint));
    elevatorMotor1.setVoltage(pidOutputBottom);
  }

  public double pidoutputBottom(int topSetpoint, int bottomSetpoint) {
    double pidOutputBottom = bottomController.calculate(bottomEncoder.getDistance(),
        Units.degreesToRadians(bottomSetpoint));
    return pidOutputBottom;
  }

  @Override
  public void periodic() {
    setHeightOnce();
     SmartDashboard.putNumber("Setpoint top (degrees)", 90);
    SmartDashboard.putNumber("Setpoint bottom (degrees)", 90);

    SmartDashboard.putData("Arm Sim", m_mech2d);
  }

    public void simulationPeriodic() {
    m_arm_topSim.setInput(elevatorMotor2.get() * RobotController.getBatteryVoltage());
    m_arm_bottomSim.setInput(elevatorMotor1.get() * RobotController.getBatteryVoltage());

    m_arm_topSim.update(0.020);
    m_arm_bottomSim.update(0.020);


    m_topEncoderSim.setDistance(m_arm_topSim.getAngleRads());
    m_bottomEncoderSim.setDistance(m_arm_bottomSim.getAngleRads());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            m_arm_topSim.getCurrentDrawAmps() + m_arm_bottomSim.getCurrentDrawAmps()));

    m_arm_top.setAngle(Units.radiansToDegrees(m_arm_topSim.getAngleRads()));
    m_arm_bottom.setAngle(Units.radiansToDegrees(m_arm_bottomSim.getAngleRads()));
  }
}
