package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;


public class ThrowObjectCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final double throwerPowerTime;
    private final double pullerPushTime;
    private final double throwerPushTime;
    private boolean pullerStarted;

    public ThrowObjectCommand(ShooterSubsystem shooterSubsystem, double throwerPowerTime, double pullerPushTime, double throwerPushTime) {
        this.shooterSubsystem = shooterSubsystem;
        this.throwerPowerTime = throwerPowerTime;
        this.pullerPushTime = pullerPushTime;
        this.throwerPushTime = throwerPushTime;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        if (!shooterSubsystem.hasObject()) {
            cancel();
            return;
        }
        pullerStarted = false;
        shooterSubsystem.runThrowerMotors(1.0);
    }
    @Override
    public void execute() {
        // Ne yazdığın hakkında zerre fikrim yok
        /*
        start_time = time.time();
        // Puller motorunun objeyi almasını bekleyin
        if (!shooterSubsystem.hasObject()) {
            return;
        }

        // Thrower motorunun tam hıza ulaşmasını bekleyin
        if (!throwerMotorAtFullSpeed()) {
            return;
        }

        // Puller motorunu çalıştırın ve objeyi fırlatın
        shooterSubsystem.runPullerMotors(1.0);

        while (time.time() - start_time < target_wait) {}

        // Fırlatma işleminin tamamlanmasını bekleyin
        if (timeSinceInitialized() < throwerPowerTime + pullerPushTime) {
            return;
        }

        // Motorları durdurun ve komutu sonlandırın
        shooterSubsystem.runPullerMotors(0.0);
        shooterSubsystem.runThrowerMotors(0.0);
        end(false);
         */
    }

    @Override
    public boolean isFinished() {
        // Command'in ne zaman bittiğini belirlemek için koşullar ekleyin
        // Örneğin, belirli bir sensör değeri okunabilir veya belirli bir süre geçebilir.
        return false; // Geçici olarak bu şekilde bırakın, kendinize göre güncelleyin
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            // Komut kesintiye uğradığında yapılacak işlemler
            shooterSubsystem.runPullerMotors(0.0);
            shooterSubsystem.runThrowerMotors(0.0);
        }
    }
}
