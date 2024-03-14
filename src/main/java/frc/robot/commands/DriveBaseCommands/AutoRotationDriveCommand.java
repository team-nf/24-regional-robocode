package frc.robot.commands;

import java.util.function.Supplier;
import java.util.function.BooleanSupplier;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AutoRotationDriveCommand extends Command{
    static int X_OFFSET_IDX;

    // Manuel döndürmeyi ortadan kaldırmak amacıyla kullanılacak
    // translation kısmı joystickden geliyor
    private final SwerveSubsystem m_drivebase;
    private final Supplier<double[]> apriltagdata;
    private final Supplier<double[]> objectdetectiondata;

    private final CommandGenericHID m_controller;
    private final BooleanSupplier m_hasObjectSupplier;

    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(25);

    AutoRotationDriveCommand(SwerveSubsystem swerve, CommandGenericHID controller, BooleanSupplier hasObject, Supplier<double[]> apriltagdata, Supplier<double[]> oddata) {
        m_drivebase = swerve;
        objectdetectiondata = oddata;
        this.apriltagdata = apriltagdata;

        m_controller = controller;
        m_hasObjectSupplier = hasObject;

        addRequirements(m_drivebase);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double rotationVelocity;
        if (m_hasObjectSupplier.getAsBoolean()) {
            rotationVelocity = apriltagdata.get()[X_OFFSET_IDX];
        } else {
            rotationVelocity = objectdetectiondata.get()[X_OFFSET_IDX];
        }
        m_drivebase.drive(
            new Translation2d(
                MathUtil.applyDeadband(m_controller.getRawAxis(OperatorConstants.LEFT_Y_AXIS), OperatorConstants.LEFT_Y_DEADBAND),
                MathUtil.applyDeadband(m_controller.getRawAxis(OperatorConstants.LEFT_X_AXIS), OperatorConstants.LEFT_X_DEADBAND)
            ),
            rotationLimiter.calculate(rotationVelocity), false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
