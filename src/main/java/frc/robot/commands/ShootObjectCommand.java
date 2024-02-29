// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.ShooterSubsystem;


// public class ShootObjectCommand extends Command {
//     private final ShooterSubsystem m_shooter;
//     private final DriveSubsystem m_drivebase;
//     private boolean noObjectOnStart;

//     public ShootObjectCommand(double angle, ShooterSubsystem shooterSubsystem, DriveSubsystem driveSubsystem) {
//         m_shooter = shooterSubsystem;
//         m_drivebase = driveSubsystem;

//         /** En başta obje yoksa command çalışmayacak */
//         noObjectOnStart = !m_shooter.hasObject();

//         addRequirements(m_shooter);
//         addRequirements(m_drivebase);
//     }

//     @Override
//     public void execute() {
//         if (noObjectOnStart) { return; }

//         if (m_drivebase.isManualRotated) {
//             /** Apriltage dön */
//             m_drivebase.drive(translation(0,0), networktables.get("at: target angle"));
//         }
//         m_shooter.setAngleCommand(networktables.get("at: throw angle"));
//         m_shooter.throwObjectCommand();
//     }

//     @Override
//     public boolean isFinished() {
//         /** En başta obje yoksa çalıştırma */
//         if (noObjectOnStart) { return true; }

//         /** En başta obje varsa ve şu anda yoksa komut bitmiş demektir */
//         return !m_shooter.hasObject();
//     }
// }
