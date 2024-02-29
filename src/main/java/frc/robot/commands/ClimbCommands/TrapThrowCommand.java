package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class TrapThrowCommand extends SequentialCommandGroup {
    private final ShooterSubsystem m_shooter;
    private final ElevatorSubsystem m_elevator;

    public TrapThrowCommand(ShooterSubsystem shooterSubsystem, ElevatorSubsystem elevatorSubsystem) {
        m_shooter = shooterSubsystem;
        m_elevator = elevatorSubsystem;

        addRequirements(m_elevator);
        addRequirements(m_shooter);

        /** Eğer elevator başlangıçta açıksa komudu çalıştırma */
        if (m_elevator.isElevatorFullyClosed() == false) {
            addCommands();
            return;
        }

        addCommands(
            m_elevator.openElevatorCommand(),
            m_shooter.trapThrowCommand(),
            m_elevator.closeElevatorCommand()
        );
    }
}
