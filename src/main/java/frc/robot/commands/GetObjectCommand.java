// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.ShooterSubsystem;


// public class GetObjectCommand extends Command {
//     private final ShooterSubsystem m_shooter;
//     private final IntakeSubsystem m_intake;

//     private boolean m_isUpperRead = false;
//     private boolean m_isFinished = false;

//     public GetObjectCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
//         m_intake = intakeSubsystem;
//         m_shooter = shooterSubsystem;

//         addRequirements(m_intake);
//         addRequirements(m_shooter);
//     }

//     /** Alttaki sensör objeyi algılamışsa ve üstteki 
//      * algılamamışsa istediğimiz konumdayız demektir */
//     public boolean checkObjectPosition() {
//         return m_shooter.getLowerSensorReading() && !m_shooter.getUpperSensorReading();
//     }

//     @Override
//     public void initialize() {
//         // m_shooter.setToIntakeAngle();
//         m_intake.stopIntake();
//         m_shooter.stopThrower();
//         m_shooter.stopFeeder();
//     }

//     @Override
//     public void execute() {
//         /* İçimizde zaten obje yoksa almaya çalış */
//         if (m_shooter.getLowerSensorReading() == false && 
//             m_shooter.getUpperSensorReading() == false
//         ) {
//             m_intake.runIntake();
//             m_shooter.runFeeder();
//         }

//         // üst sensöre geldiysek dur ve motorları ters çevir
//         else if (m_shooter.getUpperSensorReading()) {
//             m_intake.stopIntake();
//             m_shooter.runFeederReverse();
//             m_isUpperRead = true;

//             /**
//              *  Bunu burada tekrar setleme amacım, 
//              * komut bitiminde objeyi geri çekmiş olsak 
//              * da obje esnek olduğundan tekrar sensör 
//              * görebilir, böyle bir durumda objeyi 
//              * tekrar geri çekmek istemem 
//              */
//             m_isFinished = false;
//         }

//         /* Eğer daha önce üst sensör seviyesine geldiysek 
//             ve şu an sensör objeyi görmüyorsa bitti */
//         else if (m_shooter.getUpperSensorReading() == false && m_isUpperRead) {
//             m_intake.stopIntake();
//             m_shooter.stopFeeder();
//             m_isFinished = true;
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         return m_isFinished;
//     }
// }
