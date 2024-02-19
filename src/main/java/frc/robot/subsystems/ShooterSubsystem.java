package frc.robot.subsystems;

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
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax lowerThrowerMotor;
    private final CANSparkMax upperThrowerMotor;
    private final CANSparkMax pullerMotor;
    private final AbsoluteEncoder upperThrowerEncoder;
    private final AbsoluteEncoder lowerThrowerEncoder;
    private final SparkPIDController shooterAngleController;
    private final DigitalInput objectSensor;
    

    public ShooterSubsystem() {
        lowerThrowerMotor = new CANSparkMax(Constants.ShooterConstants.kShooterThrowerMotorLowerId, MotorType.kBrushless);
        upperThrowerMotor = new CANSparkMax(Constants.ShooterConstants.kShooterThrowerMotorUpperId, MotorType.kBrushless);
        upperThrowerMotor.setInverted(true);
        pullerMotor = new CANSparkMax(Constants.ShooterConstants.kShooterPullerMotorId, MotorType.kBrushless);
        upperThrowerEncoder = upperThrowerMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        lowerThrowerEncoder = lowerThrowerMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        shooterAngleController = upperThrowerMotor.getPIDController();
        shooterAngleController = lowerThrowerMotor.getPIDController();
        objectSensor = new DigitalInput(Constants.ShooterConstants.kObjectSensorPort);

        configurePID();
    }

    public boolean hasObject() { 
        return objectSensor.get();
    }

    public void runPullerMotors(double speed) {
        pullerMotor.set(speed);
    }

    public void runThrowerMotors(double speed) {
        lowerThrowerMotor.set(speed);
        upperThrowerMotor.set(speed);
    }
    public void runThrowerMotorsSpeed() {
        runThrowerMotors(Constants.ShooterConstants.kThrowerSpeed);
    }

        //ANGLE SETTER COMMAND
    public void setShooterAngle(double angle) {
        shooterAngleController.setReference(angle, ControlType.kPosition);
    }

  

    public Command getShooterAngleSetterCommand(DoubleSupplier angleSupplier) {
    return runOnce(() -> setShooterAngle(angleSupplier.getAsDouble()));
    }

        //CLIMBING MODE COMMAND
    public void setClimbingMode(double climbingAngle) {
        shooterAngleController.setReference(climbingAngle, ControlType.kPosition);
    }

    public Command getClimbingModeCommand(DoubleSupplier angleSupplier) {
        return runOnce(() -> setClimbingMode(angleSupplier.getAsDouble()));
    }

        //THROW OBJECT COMMAND
        public Command getThrowObjectCommand() {
            return new ConditionalCommand(
                new SequentialCommandGroup(
                    new WaitCommand(Constants.ShooterConstants.kPullerPushWaitTime),
                    new InstantCommand(() -> runThrowerMotorsSpeed()),
                    new InstantCommand(() -> runPullerMotors(ShooterConstants.kPullerSpeed)),
                    new WaitCommand(Constants.ShooterConstants.kThrowerPushWaitTime),
                    new InstantCommand(() -> {
                        runPullerMotors(0.0);
                        runThrowerMotors(0.0);
                    })
                ),

                new InstantCommand(() -> {
                    SmartDashboard.putString("Throw Object Status", "No object to throw!");
                    SmartDashboard.putBoolean("Has Object", false);
                }),

                () -> this::hasObject
            );
        }

  

    


    private void configurePID() {
        shooterAngleController.setP(Constants.ShooterConstants.kShooterKp);
        shooterAngleController.setI(Constants.ShooterConstants.kShooterKi);
        shooterAngleController.setD(Constants.ShooterConstants.kShooterKd);
        shooterAngleController.setFF(Constants.ShooterConstants.kShooterKf);
        shooterAngleController.setOutputRange(-1.0, 1.0);
    }



    // ÇÖP KODLAR

    // public void periodic() {
    //     shooterAngleController.calculate();
    //     throwerMotor.set(shooterAngleController.getOutput());
    // }

    
    // public void update() {
        
    
    //     // SmartDashboard'a veri göndermek için
    //     SmartDashboard.putNumber("shooter_setpoint", getSetpoint());
    //     SmartDashboard.putBoolean("shooter_on_target", isOnTarget());
    //     SmartDashboard.putNumber("shooter_current", throwerMotor.getOutputCurrent());
    // }

    
}
