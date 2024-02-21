package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Import Constants class correctly
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;




public class ShooterSubsystem extends SubsystemBase{
    /** Topu fırlatacak olan iki motordan alt */
    private final CANSparkMax lowerThrowerMotor =
        new CANSparkMax(ShooterConstants.kThrowerMotorLowerId, MotorType.kBrushless);
    private final SparkPIDController lowerThrowerController =
        lowerThrowerMotor.getPIDController();

    /** Topu fırlatacak olan iki motordan üst */
    private final CANSparkMax upperThrowerMotor =
        new CANSparkMax(ShooterConstants.kThrowerMotorUpperId, MotorType.kBrushless);
    private final SparkPIDController upperThrowerController =
        upperThrowerMotor.getPIDController();

    /** Intake kısmından gelen objeyi fırlatılacak kısma ileten motor */
    private final CANSparkMax feederMotor = new CANSparkMax(ShooterConstants.kFeederMotorId, MotorType.kBrushless);
    private final SparkPIDController feederController = feederMotor.getPIDController();

    /** Shooter açısını belirleycek motor */
    private final CANSparkMax angleMotor = new CANSparkMax(ShooterConstants.kAngleMotorId, MotorType.kBrushless);
    private final DutyCycleEncoder angleAbsoluteEncoder = new DutyCycleEncoder(ShooterConstants.kAngleEncoderId);
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


    public ShooterSubsystem() {
        /** Motor ve pid konfigürasyonları*/
        lowerThrowerController.setFeedbackDevice(lowerThrowerMotor.getEncoder());
        lowerThrowerMotor.enableVoltageCompensation(Constants.nominalVoltage);
        lowerThrowerMotor.setInverted(false);

        upperThrowerController.setFeedbackDevice(upperThrowerMotor.getEncoder());
        upperThrowerMotor.enableVoltageCompensation(Constants.nominalVoltage);
        upperThrowerMotor.setInverted(true);

        feederController.setFeedbackDevice(feederMotor.getEncoder());
        feederMotor.enableVoltageCompensation(Constants.nominalVoltage);
        feederMotor.setInverted(true);

        currentTargetAngle = angleAbsoluteEncoder.getAbsolutePosition();
        angleMotor.enableVoltageCompensation(Constants.nominalVoltage);
        angleMotor.setInverted(false);

        configurePID();
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
            () -> motor.getPIDController().setReference(targetVel, ControlType.kVelocity),
            () -> {},
            onEnd,
            hasReachedVelocity
        );
    }

    private void setAngleOnce() {
        double currentPosition = angleAbsoluteEncoder.getAbsolutePosition();
        double pidValue = angleController.calculate(currentPosition, currentTargetAngle);
        double feedforwardValue = angleFeedforward.calculate(currentTargetAngle, ShooterConstants.kAngularVel);

        angleMotor.set(pidValue + feedforwardValue);
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

    /** Feederı default hızda çalıştır ve çalışana kadar bekle */
    public Command runFeederCommand() {
        return setAndWaitMotorVelCommand(feederMotor, ShooterConstants.kFeederVelocity, 100);
    }

    /** Feederı durdur, durmasını bekleme */
    public Command stopFeederCommand() {
        return runOnce(() -> {
            feederController.setReference(0, ControlType.kVelocity);
            feederMotor.stopMotor();
        });
    }

    /** Throwerları önceden (constants dosyasında) ayarlanmış hızla çalıştır ve bekle */
    public Command runThrowerCommand() {
        return new ParallelCommandGroup(
            setAndWaitMotorVelCommand(upperThrowerMotor, ShooterConstants.kThrowerVelocity, 10),
            setAndWaitMotorVelCommand(lowerThrowerMotor, ShooterConstants.kThrowerVelocity, 10)
            /* For debugging */
            // runOnce(() -> System.out.println("RUNNING PARALLEL COMMAND GROUP"))
        );
    }

    /** Throwerları durdur, durmasını bekleme */
    public Command stopThrowerCommand() {
        return new ParallelCommandGroup(
            runOnce(()-> {
                upperThrowerController.setReference(0, ControlType.kVelocity);
                upperThrowerMotor.stopMotor();
            }),
            runOnce(()-> {
                lowerThrowerController.setReference(0, ControlType.kVelocity);
                lowerThrowerMotor.stopMotor();
            })
        );
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
        //angleController.setFF(ShooterConstants.kAngleKf);

        feederController.setP(ShooterConstants.kFeederKp);
        feederController.setI(ShooterConstants.kFeederKi);
        feederController.setD(ShooterConstants.kFeederKd);
        feederController.setFF(ShooterConstants.kFeederKf);
    }

    /**
     * Shooter açısını ayarlamak için kullanacağımız motorlarda,
     * motorun içindekinden farklı encoder kullandığımızdan, pid loopu
     * roborio tarafından yapılıyor. Bu yüzden pid loopu ayarlamak için
     * pid controolerı sürekli çağırıyoruz
     */
    @Override
    public void periodic() {
        setAngleOnce();
    }
}
