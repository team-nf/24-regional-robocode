package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkPIDController;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase; //motor
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

// Import Constants class correctly
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    /* Topu fırlatacak olan iki motor, alt ve üst */
    private final CANSparkMax lowerThrowerMotor;
    private final CANSparkMax upperThrowerMotor;

    /* Intake kısmından gelen objeyi fırlatılacak kısma ileten motor */
    private final CANSparkMax pullerMotor;

    /* Shooter açısını belirleycek motor */
    private final CANSparkMax shooterAngleMotor;
    /* Açı yönetecek motorun pidsi */
    private final SparkPIDController shooterAngleController;

    /*
     * Açıyı değiştirmek için birden fazla motor kullanılacaksa,
     * pid aynı olsun diye sparkmax pid yerine wiplib pid yapılmalı
     */
    // private final PIDController shooterAnglePIDController;

    private final DigitalInput objectSensor;

    /* Objeye sahip olup olmadığımız */
    private final boolean hasObject;


    public ShooterSubsystem(boolean hasObject) {
        /* Follewer tanımlanmalı */
        lowerThrowerMotor = new CANSparkMax(Constants.ShooterConstants.kShooterThrowerMotorLowerId, MotorType.kBrushless);
        upperThrowerMotor = new CANSparkMax(Constants.ShooterConstants.kShooterThrowerMotorUpperId, MotorType.kBrushless);
        upperThrowerMotor.setInverted(true);

        pullerMotor = new CANSparkMax(Constants.ShooterConstants.kShooterPullerMotorId, MotorType.kBrushless);

        shooterAngleMotor  = new CANSparkMax(Constants.ShooterConstants.kShooterAngleMotorId, MotorType.kBrushless);
        shooterAngleController = shooterAngleMotor.getPIDController();
        // bu hatalı. oluştururken final keyword'u verdiğin bir değişkene de verdiğin değeri değiştiremezsin.
        // shooterAngleController = lowerThrowerMotor.getPIDController();

        // shooterAnglePIDController = new PIDController(Constants.ShooterConstants.kShooterKp, Constants.ShooterConstants.kShooterKi, Constants.ShooterConstants.kShooterKd);

        objectSensor = new DigitalInput(Constants.ShooterConstants.kObjectSensorPort);
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

    public void setPullerMotors(double speed) {
        pullerMotor.set(speed);
    }

    public void setThrowerMotors(double speed) {
        lowerThrowerMotor.set(speed);
        upperThrowerMotor.set(speed);
    }

    public void runThrowerMotors() {
        setThrowerMotors(Constants.ShooterConstants.kThrowerSpeed);
    }

    public Command runThrowerMotorsCommand() {
        return run(() -> setThrowerMotors(Constants.ShooterConstants.kThrowerSpeed));
    }

    public void setShooterAngle(double angle) {
        shooterAngleController.setReference(angle, ControlType.kPosition);
    }

    public Command getShooterAngleSetterCommand(DoubleSupplier angleSupplier) {
        return runOnce(() -> setShooterAngle(angleSupplier.getAsDouble()));
    }

    public void runClimbingMode() {
        setShooterAngle(Constants.ShooterConstants.kClimbingAngle);
    }

    public Command getClimbingModeCommand() {
        return runOnce(() -> runClimbingMode());
    }

    public Command getThrowObjectCommand() {
        return new ConditionalCommand(
            new SequentialCommandGroup(
                runThrowerMotorsCommand(),
                new WaitCommand(Constants.ShooterConstants.kThrowerMaxSpeedWait),

                new InstantCommand(() -> setPullerMotors(Constants.ShooterConstants.kPullerSpeed)),
                new InstantCommand(() -> {
                    setPullerMotors(0.0);
                    setThrowerMotors(0.0);
                })
            ),

            new InstantCommand(() -> {
                SmartDashboard.putString("Throw Object Status", "No object to throw!");
                SmartDashboard.putBoolean("Has Object", false);
            }),

            () -> this.getHasObject()
        );
    }






    private void configurePID() {
        shooterAngleController.setP(Constants.ShooterConstants.kShooterKp);
        shooterAngleController.setI(Constants.ShooterConstants.kShooterKi);
        shooterAngleController.setD(Constants.ShooterConstants.kShooterKd);
        shooterAngleController.setFF(Constants.ShooterConstants.kShooterKf);
        shooterAngleController.setOutputRange(-1.0, 1.0);
    }



}
