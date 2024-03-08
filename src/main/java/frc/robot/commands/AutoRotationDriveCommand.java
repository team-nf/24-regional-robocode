package frc.robot.commands;

import java.util.function.Supplier;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AutoRotationDriveCommand extends Command{
    static int ANGLE_DATA_INDEX;

    // Manuel döndürmeyi ortadan kaldırmak amacıyla kullanılacak
    // translation kısmı joystickden geliyor
    private final ShooterSubsystem m_shooter;
    private final SwerveSubsystem m_drivebase;
    private final Supplier<double[]> apriltagdata;
    private final Supplier<double[]> objectdetectiondata;

    private final CommandGenericHID m_controller;

    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(25);

    AutoRotationDriveCommand(ShooterSubsystem shooter, SwerveSubsystem swerve, CommandGenericHID controller, Supplier<double[]> apriltagdata, Supplier<double[]> oddata) {
        m_shooter = shooter;
        m_drivebase = swerve;
        objectdetectiondata = oddata;
        this.apriltagdata = apriltagdata;

        m_controller = controller;

        addRequirements(m_shooter, m_drivebase);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Rotation2d targetAngle;
        if (m_shooter.hasObject()) {
            targetAngle = Rotation2d.fromDegrees(apriltagdata.get()[ANGLE_DATA_INDEX]);
        } else {
            targetAngle = Rotation2d.fromDegrees(objectdetectiondata.get()[ANGLE_DATA_INDEX]);
        }
        m_drivebase.drive(new Translation2d(MathUtil.applyDeadband(m_controller.getRawAxis(OperatorConstants.LEFT_Y_AXIS), OperatorConstants.LEFT_Y_DEADBAND),
        MathUtil.applyDeadband(m_controller.getRawAxis(OperatorConstants.LEFT_X_AXIS), OperatorConstants.LEFT_X_DEADBAND) ), rotationLimiter.calculate(targetAngle.getRadians()), false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    
}
