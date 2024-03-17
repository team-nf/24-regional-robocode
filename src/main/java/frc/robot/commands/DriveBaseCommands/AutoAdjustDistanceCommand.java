package frc.robot.commands.DriveBaseCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/*
 * Bu komut robotun ön-arka eksende apriltag ile arasındaki uzaklığını ayarlamasını sağlıyor
 * 
 * (Tırmanma esnasında bu komuttan önce, tırmanacağımız apriltage dik bakıyor olmalıyız.
 * Tırmanma esnasında ilk olarak AutoAdjustAngle komutu çalıştırılacak ve dik bakılma sağlanacak.
 * Bu komuttan önce AutoAdjustYCommand kullanılarak apriltagin tam karşımızda olması sağlanacak, 
 * daha sonrasında bu komut çağrılacak ve uzaklık ayarlanacak)
 * 
 */

public class AutoAdjustDistanceCommand extends Command{
    static int DISTANCE_IDX;

    private final SwerveSubsystem m_drivebase;
    private final Supplier<double[]> apriltagdata;

    private SlewRateLimiter m_velocityLimiter = new SlewRateLimiter(25);
    private final double m_adjustKP = 0.1;

    // birim cm (umarım)
    private final double m_distanceTolerance = 5;


    AutoAdjustDistanceCommand(SwerveSubsystem swerve, Supplier<double[]> apriltagdata) {
        m_drivebase = swerve;
        this.apriltagdata = apriltagdata;

        addRequirements(m_drivebase);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double velocityY = apriltagdata.get()[DISTANCE_IDX];
        double limitedVelocityY = m_velocityLimiter.calculate(velocityY) * m_adjustKP;

        m_drivebase.drive(new Translation2d(0, limitedVelocityY), 0, false);
    }

    @Override
    public boolean isFinished() {
        double distance = apriltagdata.get()[DISTANCE_IDX];

        if (-m_distanceTolerance < distance && distance < m_distanceTolerance) {
            return true;
        }

        return false;
    }
}
