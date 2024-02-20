package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Import Constants class correctly
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;




public class ShooterSubsystem extends SubsystemBase {
    /* Topu fırlatacak olan iki motor, alt ve üst */
    private final CANSparkMax lowerThrowerMotor;
    private final SparkPIDController lowerThrowerController;

    private final CANSparkMax upperThrowerMotor;
    private final SparkPIDController upperThrowerController;

    /* Intake kısmından gelen objeyi fırlatılacak kısma ileten motor */
    private final CANSparkMax feederMotor;
    private final SparkPIDController feederController;

    /* Shooter açısını belirleycek motor */
    private final CANSparkMax angleMotor;
    /* Açı yönetecek motorun pidsi */
    private final SparkPIDController angleController;
    private final DutyCycleEncoder angleAbsoluteEncoder;

    /*
     * Açıyı değiştirmek için birden fazla motor kullanılacaksa,
     * pid aynı olsun diye sparkmax pid yerine wiplib pid yapılmalı
     */
    // private final PIDController shooterAnglePIDController;

    private final DigitalInput objectSensor;

    /* Objeye sahip olup olmadığımız */
    private final boolean hasObject;


    public ShooterSubsystem(boolean hasObject) {
        /* Shooteradki fırlatan motorlardan, alttakinin tanımlamasını yap */
        lowerThrowerMotor = new CANSparkMax(ShooterConstants.kThrowerMotorLowerId, MotorType.kBrushless);
        lowerThrowerController = lowerThrowerMotor.getPIDController();
        lowerThrowerController.setFeedbackDevice(lowerThrowerMotor.getEncoder());

        /* Shooteradki fırlatan motorlardan, üsttekinin tanımlamasını yap */
        upperThrowerMotor = new CANSparkMax(ShooterConstants.kThrowerMotorUpperId, MotorType.kBrushless);
        upperThrowerController = upperThrowerMotor.getPIDController();
        upperThrowerController.setFeedbackDevice(upperThrowerMotor.getEncoder());
        upperThrowerMotor.setInverted(true);

        /* Objeyi intakeden alıp fırlatan kısma veren motorun tanımlamaları */
        feederMotor = new CANSparkMax(ShooterConstants.kFeederMotorId, MotorType.kBrushless);
        feederController = feederMotor.getPIDController();
        feederController.setFeedbackDevice(feederMotor.getEncoder());
        feederMotor.setInverted(true);

        /* Shooterın açısını ayarlayacak motorun tanımlamaları */
        angleMotor  = new CANSparkMax(ShooterConstants.kAngleMotorId, MotorType.kBrushless);
        angleAbsoluteEncoder = new DutyCycleEncoder(ShooterConstants.kAngleEncoderId);
        angleController = angleMotor.getPIDController();
        angleController.setFeedbackDevice(angleMotor.getEncoder());

        objectSensor = new DigitalInput(ShooterConstants.kObjectSensorPort);
        this.hasObject = hasObject;

        configurePID();
    }

    public ShooterSubsystem() {
        /* Başlangıçta içimizde obje olacağından
        default olarak true verebiliriz */
        this(true);
    }

    public boolean getHasObject() {
        return this.hasObject;
    }

    public Command setAndWaitMotorVelCommand(CANSparkMax motor, double targetVel, double acceptableVelError) {
        /*
         * Sets the given motor's target velocity to targetVel and waits for the motor to reach the desired velocity
         */
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

    public Command setAndWaitMotorAngleCommand(CANSparkMax motor, double targetAngle, double acceptableAngleError) {
        /*
         * Sets the given motor's target velocity to targetVel and waits for the motor to reach the desired velocity
         */
        Consumer<Boolean> onEnd = wasInterrupted -> {
            System.out.println("setAndWaitAngle ended");
        };

        BooleanSupplier hasReachedVelocity = () ->
            Math.abs(motor.getEncoder().getPosition() - targetAngle) <= acceptableAngleError;

        return new FunctionalCommand(
            () -> motor.getPIDController().setReference(targetAngle, ControlType.kPosition),
            () -> {},
            onEnd,
            hasReachedVelocity
        );
    }

    /* Feederı default hızda çalıştır ve çalışana kadar bekle */
    public Command runFeederCommand() {
        return setAndWaitMotorVelCommand(feederMotor, ShooterConstants.kFeederVelocity, 100);
    }

    /* Feederı durdur, durmasını bekleme */
    public Command stopFeederCommand() {
        return runOnce(() -> {
            feederController.setReference(0, ControlType.kVelocity);
            feederMotor.stopMotor();
        });
    }

    /* Throwerları önceden (constants dosyasında) ayarlanmış hızla çalıştır ve bekle */
    public Command runThrowerCommand() {
        return new ParallelCommandGroup(
            setAndWaitMotorAngleCommand(upperThrowerMotor, ShooterConstants.kThrowerVelocity, 10),
            setAndWaitMotorAngleCommand(lowerThrowerMotor, ShooterConstants.kThrowerVelocity, 10)
        );
    }

    /* Throwerları durdur, durmasını bekleme */
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

    /* Açıyı ayarlamaya çalış ve bitene kadar bekle */
    public Command setAngleCommand(double targetAngle, double acceptableAngleError) {
        return setAndWaitMotorAngleCommand(angleMotor, targetAngle, acceptableAngleError);
    }

    /* Açıyı ayarlamaya çalış ve bitene kadar bekle */
    public Command setAngleCommand(double targetAngle) {
        return setAngleCommand(targetAngle, 2);
    }

    public Command setClimbingAngleCommand() {
        return setAngleCommand(ShooterConstants.kClimbingAngle, 5);
    }

    public Command setIntakeAngle() {
        return setAngleCommand(ShooterConstants.kIntakeAngle, 1);
    }

    public Command throwObjectCommand() {
        return new ConditionalCommand(
            new SequentialCommandGroup(
                /* Öncelikle Thrower motorlar (neredeyse)
                    tüm hızda çalışana kadar bekle*/
                runThrowerCommand(),
                /* Daha sonra feederı çalıştır ve topu
                    fırlatıcı motorlara ver */
                runFeederCommand(),
                stopThrowerCommand(),
                stopFeederCommand()
            ),

            runOnce(() -> System.out.println("No object in shooter!!")),

            () -> this.getHasObject()
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
        angleController.setFF(ShooterConstants.kAngleKf);

        feederController.setP(ShooterConstants.kFeederKp);
        feederController.setI(ShooterConstants.kFeederKi);
        feederController.setD(ShooterConstants.kFeederKd);
        feederController.setFF(ShooterConstants.kFeederKf);
    }

}
