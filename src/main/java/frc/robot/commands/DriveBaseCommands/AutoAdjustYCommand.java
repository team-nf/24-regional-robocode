package frc.robot.commands.DriveBaseCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/*
 * Bu komut robotun sağa-sol eksende kendini apriltag ile ortalamasını sağlıyor
 * 
 * Tırmanma esnasında tırmanacağımız yerdeki apriltage bakarak kendini ayarlamasını sağlıyor
 * (Tırmanma esnasında bu komuttan önce, tırmanacağımız apriltage dik bakıyor olmalıyız. Bu 
 * sebeple önce AutoAdjustAngleCommand çalıştırılmalıdır.)
 * 
 */

public class AutoAdjustYCommand extends Command{
    static int X_OFFSET_IDX;

    private final SwerveSubsystem m_drivebase;
    private final Supplier<double[]> apriltagdata;

    private SlewRateLimiter m_velocityLimiter = new SlewRateLimiter(25);
    private final double m_adjustKP = 0.1;

    private final double m_offsetTolerance = 50;


    AutoAdjustYCommand(SwerveSubsystem swerve, Supplier<double[]> apriltagdata) {
        m_drivebase = swerve;
        this.apriltagdata = apriltagdata;

        addRequirements(m_drivebase);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double velocityY = apriltagdata.get()[X_OFFSET_IDX];
        double limitedVelocityY = m_velocityLimiter.calculate(velocityY) * m_adjustKP;

        m_drivebase.drive(new Translation2d(0, limitedVelocityY), 0, false);
    }

    @Override
    public boolean isFinished() {
        double x_offset = apriltagdata.get()[X_OFFSET_IDX];

        if (-m_offsetTolerance < x_offset && x_offset < m_offsetTolerance) {
            return true;
        }

        return false;
    }
}
