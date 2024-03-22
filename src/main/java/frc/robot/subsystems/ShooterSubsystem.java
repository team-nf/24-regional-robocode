package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxAnalogSensor.Mode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
// // import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.MutableMeasure;
// import edu.wpi.first.units.Per;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
// import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// Import Constants class correctly
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

/**
 * 
 * ÖNEMLİ
 * 
 * Shooter açısındaki Absolute Encoder yere paralel iken 0 dereceyi gösterir.
 * intake'e hızalı iken 37.5 dereceyi gösterir.
 * yere paralel konumdan yukarı çıkarken yeniden tur attığı için yeniden 43ten geri saymaya başlar haliyle üst limitimiz 3 derece.
 * 
 * Negatif voltaj shooter açısını yukarı çıkarır.
 * 
 * Değerler verilirken ve işlemler yapılırken göz önünde bulundurulmalı.
 */


public class ShooterSubsystem extends SubsystemBase {
    private double m_throwerTargetVel = 0;
    private boolean m_hasLowerReachedSetpoint = false;

    /** Topu fırlatacak olan iki motordan alt */
    public final CANSparkMax m_lowerThrowerMotor = new CANSparkMax(ShooterConstants.kLowerThrowerMotorId,
            MotorType.kBrushless);
    public final SparkPIDController m_lowerThrowerController = m_lowerThrowerMotor.getPIDController();


    private boolean m_hasUpperReachedSetpoint = false;

    /** Topu fırlatacak olan iki motordan üst */
    public final CANSparkMax m_upperThrowerMotor = new CANSparkMax(ShooterConstants.kUpperThrowerMotorId,
            MotorType.kBrushless);
    public final SparkPIDController m_upperThrowerController = m_upperThrowerMotor.getPIDController();

    
    /** Intake kısmından gelen objeyi fırlatılacak kısma ileten motor */
 //   private final WPI_TalonSRX m_feederMotor = new WPI_TalonSRX(ShooterConstants.kFeederMotorId);
    private final CANSparkMax m_feederMotor = new CANSparkMax(ShooterConstants.kFeederMotorId,MotorType.kBrushless);
    private double m_lastFeederVoltage = 0;

    private SparkAnalogSensor m_secondSensor = m_upperThrowerMotor.getAnalog(com.revrobotics.SparkAnalogSensor.Mode.kAbsolute);
    private SparkAnalogSensor m_feederSensor = m_feederMotor.getAnalog(com.revrobotics.SparkAnalogSensor.Mode.kAbsolute);

    /** Shooter açısını belirleycek motor */
    private final WPI_VictorSPX m_angleMotor = new WPI_VictorSPX(ShooterConstants.kAngleMotor1Id);
    private final SparkAbsoluteEncoder m_angleAbsoluteEncoder = m_lowerThrowerMotor.getAbsoluteEncoder(Type.kDutyCycle);
    private final double m_absoluteAngleConversionFactor = 42.0;

    // OFFSET TYPE IS ANGLE NOT RAW DATA (this is added to the angle reading)
    private final double m_absoluteAngleOffset = 50.97;

    private boolean m_hasAngleReachedSetpoint = false;

    private SlewRateLimiter m_angleLimiter = new SlewRateLimiter(ShooterConstants.kAngleRampRate);

    private final PIDController m_angleController = new PIDController(
            Constants.ShooterConstants.kAngleKp,
            Constants.ShooterConstants.kAngleKi,
            Constants.ShooterConstants.kAngleKd);

    /* In C++
     * kS and kG should have units of volts,
     * kV should have units of volts * seconds / radians,
     * and kA should have units of volts * seconds^2 / radians.
     * WPILibJ does not have a type-safe unit system.
     * We use 
     */
    private final ArmFeedforward m_angleFeedforward = new ArmFeedforward(
            Constants.ShooterConstants.kAngleKs,
            Constants.ShooterConstants.kAngleKg,
            Constants.ShooterConstants.kAngleKv);

    private double m_currentTargetAngle;

    private final DigitalInput m_upperObjectSensor = new DigitalInput(ShooterConstants.kLowerObjectSensorPort);
    private final DigitalInput m_lowerObjectSensor = new DigitalInput(ShooterConstants.kUpperObjectSensorPort);
    

    /** System Identification */
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = MutableMeasure.mutable(Units.Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Angle> m_angle = MutableMeasure.mutable(Units.Rotations.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid
    // reallocation.
    private final MutableMeasure<Velocity<Angle>> m_velocity = MutableMeasure.mutable(Units.RotationsPerSecond.of(0));

    private SysIdRoutine sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Units.Volts.per(Units.Seconds).of(ShooterConstants.kSysIdAngleRampRate),
                    Units.Volts.of(ShooterConstants.kSysIdAngleStepVoltage),
                    Units.Seconds.of(ShooterConstants.kSysIdAngleTimeout)),
            new SysIdRoutine.Mechanism(
                    drive -> {
                        m_angleMotor.setVoltage(this.voltageFilter(drive.magnitude()));
                    },
                    log -> {
                        // Record a frame for the shooter motor.
                        log.motor("shooter-angle")
                                .voltage(
                                        m_appliedVoltage.mut_replace(
                                                m_angleMotor.getMotorOutputVoltage(), Units.Volts))
                                .angularPosition(m_angle.mut_replace(m_angleAbsoluteEncoder.getPosition(), Units.Rotations))
                                .angularVelocity(m_velocity.mut_replace(m_angleAbsoluteEncoder.getVelocity(), Units.RotationsPerSecond));
                    },
                    this));


    public ShooterSubsystem() {
        /** Motor ve pid konfigürasyonları */
        m_lowerThrowerController.setFeedbackDevice(m_lowerThrowerMotor.getEncoder());
        // lowerThrowerMotor.enableVoltageCompensation(Constants.nominalVoltage);
        m_lowerThrowerMotor.setInverted(true);

        m_upperThrowerController.setFeedbackDevice(m_upperThrowerMotor.getEncoder());
        // upperThrowerMotor.enableVoltageCompensation(Constants.nominalVoltage);
        m_upperThrowerMotor.setInverted(false);

        m_feederMotor.setInverted(true);

        m_angleMotor.setInverted(true);
        // m_angleMotor.configForwardSoftLimitThreshold()

        //m_angleAbsoluteEncoder.setPositionConversionFactor(43.);
        //m_angleAbsoluteEncoder.setInverted(false);
        // m_angleAbsoluteEncoder.setDistancePerRotation(42.0);
        // m_angleAbsoluteEncoder.reset();

        m_currentTargetAngle = getAngle();


        configurePID();
        updateSmartDashboard();
    }

    public double getCurrentAngleSetpoint() {
        return m_currentTargetAngle;
    }

    private void updateSmartDashboard() {
        SmartDashboard.putBoolean("Has lower thrower reached setpoint", m_hasLowerReachedSetpoint);
        SmartDashboard.putBoolean("Has upper thrower reached target", m_hasUpperReachedSetpoint);
        SmartDashboard.putNumber("Feeder Voltage", m_lastFeederVoltage);
        SmartDashboard.putNumber("Thrower Target Vel", m_lastFeederVoltage);
        SmartDashboard.putNumber("Upper Thrower Current Vel", m_upperThrowerMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Lower Thrower Current Vel", m_lowerThrowerMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter Angle ", getAngle());
        SmartDashboard.putNumber("Throughbore Raw Reading", m_angleAbsoluteEncoder.getPosition());
        SmartDashboard.putNumber("Throughbore Position Conversion Factor", m_angleAbsoluteEncoder.getPositionConversionFactor());
        SmartDashboard.putNumber("Through Bore zero offset", m_angleAbsoluteEncoder.getZeroOffset());
        SmartDashboard.putNumber("Through Bore", m_angleAbsoluteEncoder.getPosition());

        // SmartDashboard.putData("Throughbore", m_angleAbsoluteEncoder);
        // SmartDashboard.putNumber("Current Limit", m_feederMotor.getSupplyCurrent());
        SmartDashboard.putNumber("Current Limit", m_feederMotor.getOutputCurrent());
        SmartDashboard.putNumber("Feeder Analog Object Position", m_feederSensor.getPosition());
        SmartDashboard.putNumber("Feeder Analog Object 2 Position", m_secondSensor.getPosition());

       
    }

    private void configurePID() {
        m_upperThrowerController.setP(ShooterConstants.kUpperThrowerKp);
        m_upperThrowerController.setI(ShooterConstants.kUpperThrowerKi);
        m_upperThrowerController.setD(ShooterConstants.kUpperThrowerKd);
        m_upperThrowerController.setFF(ShooterConstants.kUpperThrowerKf);

        m_lowerThrowerController.setP(ShooterConstants.kLowerThrowerKp);
        m_lowerThrowerController.setI(ShooterConstants.kLowerThrowerKi);
        m_lowerThrowerController.setD(ShooterConstants.kLowerThrowerKd);
        m_lowerThrowerController.setFF(ShooterConstants.kLowerThrowerKf);

        m_angleController.setP(ShooterConstants.kAngleKp);
        m_angleController.setI(ShooterConstants.kAngleKi);
        m_angleController.setD(ShooterConstants.kAngleKd);
    }

    private double calculateFF(double angleSetpoint, double velocitySetpoint) {
        return calculateFF(angleSetpoint, velocitySetpoint, 0.0);
    }

    private double calculateFF(double angleSetpoint, double velocitySetpoint, double accelerationSetpoint) {
        return -m_angleFeedforward.calculate(angleSetpoint, velocitySetpoint, accelerationSetpoint);
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
        // return getLowerSensorReading() || getUpperSensorReading();
      //  return m_feederMotor.getSupplyCurrent() > ShooterConstants.currentlimit; 

      return m_feederMotor.getOutputCurrent() > ShooterConstants.currentlimit; 
      
    }

    /**
     * Objenin alt sensör tarafından algılanıp algılanmadığı
     * @return
     */
    public boolean getLowerSensorReading() {
        return m_lowerObjectSensor.get() == false;
    }

    /**
     * Objenin üst sensör tarafından algılanıp algılanmadığı
     * @return
     */
    public boolean getUpperSensorReading() {
        return m_upperObjectSensor.get() == false;
    }

    /**
     * Sets the given motor's target velocity to targetVel and waits for the motor
     * to reach the desired velocity
     */
    // public Command setAndWaitMotorVelCommand(CANSparkMax motor, double targetVel, double acceptableVelError) {
    //     Consumer<Boolean> onEnd = wasInterrupted -> {
    //         // System.out.println("setAndWaitVel ended");
    //     };

    //     BooleanSupplier hasReachedVelocity = () -> Math
    //             .abs(motor.getEncoder().getVelocity() - targetVel) <= acceptableVelError;

    //     return new FunctionalCommand(
    //             () -> {},
    //             () -> motor.getPIDController().setReference(targetVel, ControlType.kVelocity),
    //             onEnd,
    //             hasReachedVelocity);
    // }

    //! THROWER 
    //! THROWER 
    //! THROWER 
    //! THROWER 
    //! THROWER 
    //! THROWER 

    /**
     * Sets both motors' target velocity to targetVel and waits for the motors to
     * reach the desired velocity
     */
    public Command setShooterVelCommand(double targetVel, double acceptableVelError) {
        Consumer<Boolean> onEnd = wasInterrupted -> {
            // System.out.println("setAndWaitVel ended");
        };

        BooleanSupplier hasReachedVelocity = () -> {
            m_throwerTargetVel = targetVel;
            m_hasUpperReachedSetpoint = Math
                    .abs(m_upperThrowerMotor.getEncoder().getVelocity() - m_throwerTargetVel) <= acceptableVelError;
            m_hasLowerReachedSetpoint = Math
                    .abs(m_lowerThrowerMotor.getEncoder().getVelocity() - m_throwerTargetVel) <= acceptableVelError;

            return m_hasUpperReachedSetpoint && m_hasLowerReachedSetpoint;
        };

        return new FunctionalCommand(
                () -> {
                    // System.out.println("setShooterVel inner Command init.");
                },
                () -> {
                    m_throwerTargetVel = targetVel;
                    m_lowerThrowerController.setReference(m_throwerTargetVel, ControlType.kVelocity);
                    m_upperThrowerController.setReference(m_throwerTargetVel, ControlType.kVelocity);
                },
                onEnd,
                hasReachedVelocity);
    }

    public void setShooterVelOnce(double speed) {
        m_upperThrowerController.setReference(speed, ControlType.kVelocity);
        m_lowerThrowerController.setReference(speed, ControlType.kVelocity);
    }

    public Command setShooterVelOnceCommand(DoubleSupplier speed) {
        return runOnce(() -> {
            setShooterVelOnce(speed.getAsDouble());
        });
    }

    /**
     * Throwerları önceden (constants dosyasında) ayarlanmış hızla çalıştır ve bekle
     */
    public Command runThrowerCommand() {
        return setShooterVelCommand(ShooterConstants.kThrowerVelocity, ShooterConstants.kThrowerVelError);
    }

    public void stopThrower() {
        m_throwerTargetVel = 0;
        m_upperThrowerController.setReference(0, ControlType.kVelocity);
        m_lowerThrowerController.setReference(0, ControlType.kVelocity);
        m_upperThrowerMotor.stopMotor();
        m_lowerThrowerMotor.stopMotor();
    }

    /** Throwerları durdur, durmasını bekleme */
    public Command stopThrowerCommand() {
        return runOnce(() -> {
            stopThrower();
        });
    }


    //! ANGLE
    //! ANGLE
    //! ANGLE
    //! ANGLE
    //! ANGLE
    //! ANGLE


    public double getAngle() {
        if (m_angleAbsoluteEncoder.getPosition() < 40) {
            return m_angleAbsoluteEncoder.getPosition();
        }    
        return 43 - m_angleAbsoluteEncoder.getPosition();
    }

    private void setAngleOnce() {
        double currentPosition = m_angleAbsoluteEncoder.getPosition();
        double pidValue = m_angleController.calculate(currentPosition, m_currentTargetAngle);
        double feedforwardValue = m_angleFeedforward.calculate(m_currentTargetAngle, ShooterConstants.kAngularVel);

        m_angleMotor.setVoltage(voltageFilter(pidValue + feedforwardValue));
    }

    private void setAngleOnce(double targetAngle) {
        m_currentTargetAngle = targetAngle;
        setAngleOnce();
    }

    /**
     * Sets the angle of the shooter to the given target angle and waits for the
     * motor to reach the desired angle
     */
    public Command setAngleCommand(double targetAngle, double acceptableAngleError) {
        Consumer<Boolean> onEnd = wasInterrupted -> {
            // System.out.println("setAngleCommand ended");
        };

        BooleanSupplier hasReachedAngle = () -> {
            m_hasAngleReachedSetpoint = Math.abs(m_angleAbsoluteEncoder.getPosition() - targetAngle) <= acceptableAngleError;
            return m_hasAngleReachedSetpoint;
        };

        return new FunctionalCommand(
                () -> setAngleOnce(targetAngle),
                () -> {},
                onEnd,
                hasReachedAngle);
    }

    /** Açıyı ayarlamaya çalış ve bitene kadar bekle */
    public Command setAngleCommand(double targetAngle) {
        return setAngleCommand(targetAngle, Constants.ShooterConstants.kAngleToleranceRPS);
    }

    public void setAngleMotorVoltage(double voltage) {
        m_angleMotor.setVoltage(voltageFilter(voltage));
    }

    /**
     * Filters the voltage thats going to be given to the shooters angle adjusting motor.
     * Should stop the shooter from damaging hardware.
     * Use whenever giving voltage to the shooters angle adjuster.
     * 
     * @param voltage voltage to filter
     * @return if hit limit only voltage needed to keep the shooter in place. 
     */
    public double voltageFilter(double voltage) {
        if (voltage == 0) {return 0;}
        if (m_angleAbsoluteEncoder.getPosition() >= 40 && voltage < calculateFF(getAngle(), 0)) {
            return calculateFF(getAngle(), 0);
        }
           if (m_angleAbsoluteEncoder.getPosition() >= 40 &&m_angleAbsoluteEncoder.getPosition() <= 43 && voltage < calculateFF(getAngle(), 0)) {
            return calculateFF(getAngle(), 0);
        }
        if (m_angleAbsoluteEncoder.getPosition() >= 37.5 && getAngle() < 40 && voltage > calculateFF(getAngle(), 0)) {
            return calculateFF(getAngle(), 0);
        }
        return m_angleLimiter.calculate(voltage);
    }

    public Command setClimbingAngleCommand() {
        return setAngleCommand(ShooterConstants.kClimbingAngle, 5);
    }

    public Command setToIntakeAngle() {
        return setAngleCommand(ShooterConstants.kIntakeAngle, 1);
    }


    //! FEEDER
    //! FEEDER
    //! FEEDER
    //! FEEDER
    //! FEEDER
    //! FEEDER


    public void runFeeder()         { setFeederVoltage(ShooterConstants.kFeederVoltage); }
    public void runFeederReverse()  { setFeederVoltage(-ShooterConstants.kFeederReverseVoltage); }
    public void stopFeeder()        { setFeederVoltage(0); }

    public Command runFeederCommand()           { return runOnce(() -> runFeeder()); }
    public Command runFeederReverseCommand()    { return runOnce(() -> runFeederReverse()); }
    public Command stopFeederCommand()          { return runOnce(() -> stopFeeder()); }

    /**
     * Feeder içinde bulunan bir objeyi, önce thrower
     * motorları hızlandırarak düzgün bir şekilde fırlat
     */
    public Command throwObjectCommand() {
        return new ConditionalCommand(
                new SequentialCommandGroup(
                        /*
                         * Öncelikle Thrower motorlar (neredeyse)
                         * tüm hızda çalışana kadar bekle
                         */
                        runThrowerCommand(),
                        /*
                         * Daha sonra feederı çalıştır ve topu
                         * fırlatıcı motorlara ver
                         */
                        runFeederCommand(),
                        new WaitCommand(3),
                        stopFeederCommand(),
                        stopThrowerCommand()),

                runOnce(() -> System.out.println("No object in shooter, cannot throw")),

                () -> this.hasObject()
            );
    }

    public void setFeederVoltage(double voltage) {
        m_lastFeederVoltage = voltage;
        setFeederVoltage();
    }

    public void setFeederVoltage() {
        m_feederMotor.setVoltage(m_lastFeederVoltage);
    }

    public Command trapThrowCommand() {
        return new SequentialCommandGroup(
                setAngleCommand(ShooterConstants.kTrapThrowAngle, 3),
                runFeederReverseCommand());
    }

    /** 
     * Condition method for shooter angle motor. 
     * Returns false if limits are not reached.
     * Returns true for both forward and reverse limit.
     * Does not control any motion setpoint or setpoint direction. 
     */
    public boolean hitSoftLimit() {
        if (getAngle() <= -25) {
            return true;
        }
        if (getAngle() >= 35) {
            return true;
        }
        return false;
    }

    /**
     * Returns a command that will execute a quasistatic test in the given
     * direction.
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
        // setAngleOnce();

        updateSmartDashboard();
    }
}
