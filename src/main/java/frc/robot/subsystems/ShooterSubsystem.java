// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkRelativeEncoder;
// import com.revrobotics.SparkAbsoluteEncoder;
// import com.revrobotics.CANSparkBase; //motor
// import com.revrobotics.CANSparkBase.ControlType;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// // Import Constants class correctly
// import frc.robot.Constants;

// public class ShooterSubsystem extends SubsystemBase {
//     CANSparkMax shooterMotor;
//     SparkAbsoluteEncoder shooterEncoder;

//     public ShooterSubsystem() {
//         shooterMotor = new CANSparkMax(Constants.ShooterConstants.kShooterMotorId, MotorType.kBrushless);
//         shooterEncoder = getEncoder(SparkRelativeEncoder.Type ,  int) 

//         shooterMotor.setSmartCurrentLimit(Constants.ShooterConstants.kShooterCurrentLimit);
//         shooterMotor.setClosedLoopRampRate(Constants.ShooterConstants.kShooterRampRate);
//         shooterMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

//         // Note: burnFlash() is deprecated in newer WPILib versions
//         // shooterMotor.burnFlash();
//     }

//     public synchronized double getRpm() {
//         return shooterEncoder.getVelocity();
//     }

//     public synchronized void setRpm(double rpm) {
//         shooterMotor.getPIDController().setReference(rpm, ControlType.kVelocity);
//     }

//     public synchronized void setOpenLoop(double speed) {
//         shooterMotor.set(speed);
//     }

//     public synchronized double getSetpoint() {
//         return shooterMotor.get();
//     }

//     public synchronized boolean isOnTarget() {
//         return Math.abs(getRpm() - getSetpoint()) < Constants.ShooterConstants.kShooterOnTargetTolerance;
//     }

//     @Override
//     public synchronized void stop() {
//         setOpenLoop(0);
//     }

//     @Override
//     public void outputToSmartDashboard() {
//         SmartDashboard.putNumber("shooter_rpm", getRpm());
//         SmartDashboard.putNumber("shooter_setpoint", getSetpoint());
//         SmartDashboard.putBoolean("shooter_on_target", isOnTarget());
//         SmartDashboard.putNumber("shooter_current", shooterMotor.getOutputCurrent());
//     }

//     @Override
//     public void zeroSensors() {
//         // no-op
//     }
// }
