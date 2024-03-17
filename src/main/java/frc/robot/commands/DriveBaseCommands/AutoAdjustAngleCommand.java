package frc.robot.commands.DriveBaseCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


//! DÜZENLEME YAPILMALI
//! DÜZENLEME YAPILMALI
//! DÜZENLEME YAPILMALI
//! DÜZENLEME YAPILMALI
//! DÜZENLEME YAPILMALI
//! DÜZENLEME YAPILMALI
//! DÜZENLEME YAPILMALI
//! DÜZENLEME YAPILMALI
//! DÜZENLEME YAPILMALI
//! DÜZENLEME YAPILMALI
//! DÜZENLEME YAPILMALI
//! DÜZENLEME YAPILMALI
//! DÜZENLEME YAPILMALI


/*
 * Bu komut robotun apriltage dik (ya da başka bir açıyla) bakamasını sağlar.
 * 
 * Tırmanma esnasında tırmanacağımız yerdeki apriltage bakarak kendini ayarlamasını sağlıyor
 * (Tırmanma esnasında bu komuttan sonra AutoAdjustYCommand çalıştırılmalıdır.)
 * 
 */

public class AutoAdjustAngleCommand extends Command{
    static int X_ANGLE_IDX;

    private final SwerveSubsystem m_drivebase;
    private final Supplier<double[]> apriltagdata;

    private SlewRateLimiter m_velocityLimiter = new SlewRateLimiter(25);
    private final double m_adjustKP = 0.1;

    private final double m_angleTolerance = 50;


    AutoAdjustAngleCommand(SwerveSubsystem swerve, Supplier<double[]> apriltagdata) {
        m_drivebase = swerve;
        this.apriltagdata = apriltagdata;

        addRequirements(m_drivebase);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double rotationVelocity = apriltagdata.get()[X_ANGLE_IDX];
        double limitedRotationVelocity = m_velocityLimiter.calculate(rotationVelocity) * m_adjustKP;

        m_drivebase.drive(new Translation2d(0, 0), limitedRotationVelocity, false);
    }

    @Override
    public boolean isFinished() {
        double angle = apriltagdata.get()[X_ANGLE_IDX];

        if (-m_angleTolerance < angle && angle < m_angleTolerance) {
            return true;
        }

        return false;
    }
}
