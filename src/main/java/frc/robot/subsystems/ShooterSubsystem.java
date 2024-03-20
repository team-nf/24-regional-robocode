package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
// import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Per;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// Import Constants class correctly
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;



public class ShooterSubsystem extends SubsystemBase{
    /** Topu fırlatacak olan iki motordan alt */
    public final CANSparkMax lowerThrowerMotor =
        new CANSparkMax(ShooterConstants.kThrowerMotorLowerId, MotorType.kBrushless);
    public final SparkPIDController lowerThrowerController =
        lowerThrowerMotor.getPIDController();

    /** Topu fırlatacak olan iki motordan üst */
    public final CANSparkMax upperThrowerMotor =
        new CANSparkMax(ShooterConstants.kThrowerMotorUpperId, MotorType.kBrushless);
    public final SparkPIDController upperThrowerController =
        upperThrowerMotor.getPIDController();

    /** Intake kısmından gelen objeyi fırlatılacak kısma ileten motor */
    private final WPI_VictorSPX feederMotor = new WPI_VictorSPX(ShooterConstants.kFeederMotorId);
    private double m_lastFeederVoltage = 0;
    
    /** Shooter açısını belirleycek motor */
    private final WPI_VictorSPX angleMotor1 = new WPI_VictorSPX(ShooterConstants.kAngleMotor1Id);
    private final DutyCycleEncoder angleAbsoluteEncoder = new DutyCycleEncoder(ShooterConstants.kAngleEncoderId);
    

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = MutableMeasure.mutable(Units.Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Angle> m_angle = MutableMeasure.mutable(Units.Rotations.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Angle>> m_velocity = MutableMeasure.mutable(Units.RotationsPerSecond.of(0));

    private SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Units.Volts.per(Units.Seconds).of(ShooterConstants.kAngleRampRate), 
            Units.Volts.of(ShooterConstants.kAngleStepVoltage),
            Units.Seconds.of(ShooterConstants.kSysIdTimeout)),
        new SysIdRoutine.Mechanism(
            drive -> {
                angleMotor1.setVoltage(drive.magnitude());
            }, 
            log -> {
                // Record a frame for the shooter motor.
                log.motor("shooter-angle")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            angleMotor1.getMotorOutputVoltage(), Units.Volts))
                    .angularPosition(m_angle.mut_replace(angleAbsoluteEncoder.getAbsolutePosition() - angleAbsoluteEncoder.getPositionOffset(), Units.Rotations))
                    .angularVelocity(m_velocity.mut_replace(angleAbsoluteEncoder.getAbsolutePosition() - angleAbsoluteEncoder.getPositionOffset() - m_velocity.baseUnitMagnitude(), Units.RotationsPerSecond));
            }, 
            this));

    private final PIDController angleController = new PIDController(
        Constants.ShooterConstants.kAngleKp,
        Constants.ShooterConstants.kAngleKi,
        Constants.ShooterConstants.kAngleKd
    );

    /*
     * kS and kG should have units of volts,
     * kV should have units of volts * seconds / radians,
     * and kA should have units of volts * seconds^2 / radians.
     * WPILibJ does not have a type-safe unit system.
     */
    private final ArmFeedforward angleFeedforward = new ArmFeedforward(
        Constants.ShooterConstants.kAngleKs,
        Constants.ShooterConstants.kAngleKg,
        Constants.ShooterConstants.kAngleKv
    );
    private double currentTargetAngle;

    private final DigitalInput objectSensor = new DigitalInput(ShooterConstants.kObjectSensorPort);

    private boolean hasLowerReachedSetpoint = false;
    private boolean hasUpperReachedSetpoint = false;

    public ShooterSubsystem() {
        SmartDashboard.putBoolean("Has lower reached setpoint", hasLowerReachedSetpoint);
        SmartDashboard.putBoolean("Has upper reached setpoint", hasUpperReachedSetpoint);
        SmartDashboard.putNumber("Feeder Voltage", m_lastFeederVoltage);

        /** Motor ve pid konfigürasyonları*/
        lowerThrowerController.setFeedbackDevice(lowerThrowerMotor.getEncoder());
        //lowerThrowerMotor.enableVoltageCompensation(Constants.nominalVoltage);
        lowerThrowerMotor.setInverted(true);

        upperThrowerController.setFeedbackDevice(upperThrowerMotor.getEncoder());
        //upperThrowerMotor.enableVoltageCompensation(Constants.nominalVoltage);
        upperThrowerMotor.setInverted(false);

        feederMotor.setInverted(true);

        // feederController.setFeedbackDevice(feederMotor.getEncoder());
        // feederMotor.enableVoltageCompensation(Constants.nominalVoltage);
        // feederMotor.setInverted(true);

        currentTargetAngle = angleAbsoluteEncoder.getAbsolutePosition();
        //angleMotor1.configVoltageCompSaturation(Constants.nominalVoltage);
        //angleMotor1.enableVoltageCompensation(true);

        //feederMotor.configVoltageCompSaturation(Constants.nominalVoltage);
        //feederMotor.enableVoltageCompensation(true);

        //angleMotor1.configReverseSoftLimitThreshold(0);
        //angleMotor1.configForwardSoftLimitEnable(true);
        //angleMotor1.configReverseSoftLimitThreshold(0.5);
        //angleMotor1.configReverseSoftLimitEnable(true);

        configurePID();
    }

    private void configurePID() {
        upperThrowerController.setP(ShooterConstants.kUpperThrowerKp);
        upperThrowerController.setI(ShooterConstants.kUpperThrowerKi);
        upperThrowerController.setD(ShooterConstants.kUpperThrowerKd);
        upperThrowerController.setFF(ShooterConstants.kUpperThrowerKf);

        lowerThrowerController.setP(ShooterConstants.kLowerThrowerKp);
        lowerThrowerController.setI(ShooterConstants.kLowerThrowerKi);
        lowerThrowerController.setD(ShooterConstants.kLowerThrowerKd);
        lowerThrowerController.setFF(ShooterConstants.kLowerThrowerKf);

        angleController.setP(ShooterConstants.kAngleKp);
        angleController.setI(ShooterConstants.kAngleKi);
        angleController.setD(ShooterConstants.kAngleKd);
    }

    /**
     * Condition method
     *
     * @return True has object, false if opposing.
     */
    public boolean hasObject() {
        /**
         * Rumeysanın dediği gibi yaptım
         * benimkini geliştirmeye üşendim
         */
        return objectSensor.get();
    }

    /**
     * Sets the given motor's target velocity to targetVel and waits for the motor to reach the desired velocity
     */
    public Command setAndWaitMotorVelCommand(CANSparkMax motor, double targetVel, double acceptableVelError) {
        Consumer<Boolean> onEnd = wasInterrupted -> {
            System.out.println("setAndWaitVel ended");
        };

        BooleanSupplier hasReachedVelocity = () ->
            Math.abs(motor.getEncoder().getVelocity() - targetVel) <= acceptableVelError;

        return new FunctionalCommand(
            () -> {},
            () -> motor.getPIDController().setReference(targetVel, ControlType.kVelocity),
            onEnd,
            hasReachedVelocity
        );
    }

    /**
     * Sets both motors' target velocity to targetVel and waits for the motors to reach the desired velocity
     */
    public Command setAndWaitShooterVelCommand(double targetVel, double acceptableVelError) {
        Consumer<Boolean> onEnd = wasInterrupted -> {
            System.out.println("setAndWaitVel ended");
        };

        BooleanSupplier hasReachedVelocity = () -> {
            hasUpperReachedSetpoint = Math.abs(upperThrowerMotor.getEncoder().getVelocity() - targetVel) <= acceptableVelError;
            hasLowerReachedSetpoint = Math.abs(lowerThrowerMotor.getEncoder().getVelocity() - targetVel) <= acceptableVelError;

            return hasUpperReachedSetpoint && hasLowerReachedSetpoint;
        };

        return new FunctionalCommand(
            () -> {},
            () -> {
                lowerThrowerController.setReference(targetVel, ControlType.kVelocity);
                upperThrowerController.setReference(targetVel, ControlType.kVelocity);
            },
            onEnd,
            hasReachedVelocity
        );
    }

    public void setShooterSpeed(double speed) {
        upperThrowerController.setReference(speed, ControlType.kVelocity);
        lowerThrowerController.setReference(speed, ControlType.kVelocity);
    }

    // public Command debugVoltage(double voltage) {
    //     return runEnd( () -> {
    //         upperThrowerController.setReference(voltage, ControlType.kVoltage); 
    //         lowerThrowerController.setReference(voltage, ControlType.kVoltage);
    //     }, this::stopThrowerCommand);
    // }

    private void setAngleOnce() {
        double currentPosition = angleAbsoluteEncoder.getAbsolutePosition();
        double pidValue = angleController.calculate(currentPosition, currentTargetAngle);
        double feedforwardValue = angleFeedforward.calculate(currentTargetAngle, ShooterConstants.kAngularVel);

        angleMotor1.set(pidValue + feedforwardValue);
    }

    private void setAngleOnce(double targetAngle) {
        this.currentTargetAngle = targetAngle;
        setAngleOnce();
    }

    /** Sets the angle of the shooter to the given target angle and waits for the motor to reach the desired angle */
    public Command setAngleCommand(double targetAngle, double acceptableAngleError) {
        Consumer<Boolean> onEnd = wasInterrupted -> {
            System.out.println("setAngleCommand ended");
        };

        BooleanSupplier hasReachedAngle = () ->
            Math.abs(angleAbsoluteEncoder.getAbsolutePosition() - targetAngle) <= acceptableAngleError;

        return new FunctionalCommand(
            () -> {},
            () -> setAngleOnce(targetAngle),
            onEnd,
            hasReachedAngle
        );
    }

    /** Açıyı ayarlamaya çalış ve bitene kadar bekle */
    public Command setAngleCommand(double targetAngle) {
        return setAngleCommand(targetAngle, Constants.ShooterConstants.kAngleToleranceRPS);
    }

    public Command setClimbingAngleCommand() {
        return setAngleCommand(ShooterConstants.kClimbingAngle, 5);
    }

    public Command setIntakeAngle() {
        return setAngleCommand(ShooterConstants.kIntakeAngle, 1);
    }

    //! FEEDER
    //! FEEDER
    //! FEEDER

    /** Feederı default hızda çalıştır ve çalışana kadar bekle */
    public Command runFeederCommand() {
        return runOnce(() -> { setFeederVoltage(ShooterConstants.kFeederReverseVoltage); });
    }

    /** Feederı default hızda çalıştır ve çalışana kadar bekle */
    public Command runFeederReverseCommand() {
        return runOnce(() -> { setFeederVoltage(-ShooterConstants.kFeederReverseVoltage); });
    }

    /** Feederı durdur, durmasını bekleme */
    public Command stopFeederCommand() {
        return runOnce(() -> { setFeederVoltage(0); });
    }

    /** Throwerları önceden (constants dosyasında) ayarlanmış hızla çalıştır ve bekle */
    public Command runThrowerCommand() {
        return setAndWaitShooterVelCommand(ShooterConstants.kThrowerVelocity, 10);
    }

    public void stopThrower() {
        upperThrowerController.setReference(0, ControlType.kVelocity);
        lowerThrowerController.setReference(0, ControlType.kVelocity);
        upperThrowerMotor.stopMotor();
        lowerThrowerMotor.stopMotor();
    }

    /** Throwerları durdur, durmasını bekleme */
    public Command stopThrowerCommand() {
        return runOnce(()-> { stopThrower(); });
    }

    /**
     * Feeder içinde bulunan bir objeyi, önce thrower
     * motorları hızlandırarak düzgün bir şekilde fırlat
     */
    public Command throwObjectCommand() {
        return new ConditionalCommand(
            new SequentialCommandGroup(
                /* Öncelikle Thrower motorlar (neredeyse)
                    tüm hızda çalışana kadar bekle*/
                runThrowerCommand(),
                /* Daha sonra feederı çalıştır ve topu
                    fırlatıcı motorlara ver */
                runFeederCommand(),
                stopFeederCommand(),
                stopThrowerCommand()
            ),

            runOnce(() -> System.out.println("No object in shooter!!")),

            () -> this.hasObject()
        );
    }

    public void setFeederVoltage(double voltage) {
        m_lastFeederVoltage = voltage;
        setFeederVoltage();
    }

    public void setFeederVoltage() {
        feederMotor.setVoltage(m_lastFeederVoltage);
    }


    public Command trapThrowCommand() {
        return new SequentialCommandGroup(
            setAngleCommand(ShooterConstants.kTrapThrowAngle, 3),
            runFeederReverseCommand()
        );
    }

    /**
     * Returns a command that will execute a quasistatic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
      return sysIdRoutine.quasistatic(direction);
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return sysIdRoutine.dynamic(direction);
    }

    /**
     * Shooter açısını ayarlamak için kullanacağımız motorlarda,
     * motorun içindekinden farklı encoder kullandığımızdan, pid loopu
     * roborio tarafından yapılıyor. Bu yüzden pid loopu ayarlamak için
     * pid controolerı sürekli çağırıyoruz
     */
    @Override
    public void periodic() {
        setFeederVoltage();
        setAngleOnce();

        SmartDashboard.putBoolean("Has lower reached setpoint", hasLowerReachedSetpoint);
        SmartDashboard.putBoolean("Has upper reached setpoint", hasUpperReachedSetpoint);
        SmartDashboard.putNumber("Feeder Voltage", m_lastFeederVoltage);
    }
}
