package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class TrapThrowCommand extends Command {
    private final ShooterSubsystem m_shooter;
    private final ElevatorSubsystem m_elevator;
    private boolean wasElevatorOpenAtStart;

    public TrapThrowCommand(double angle, ShooterSubsystem shooterSubsystem, ElevatorSubsystem elevatorSubsystem) {
        m_shooter = shooterSubsystem;
        m_elevator = elevatorSubsystem;

        addRequirements(m_elevator);
        addRequirements(m_shooter);

        /** Eğer elevator başlangıçta açıksa komudu çalıştırma */
        wasElevatorOpenAtStart = m_elevator.isElevatorOpen();
    }

    @Override
    public void initialize() {
        if (wasElevatorOpenAtStart) { return; }
    }

    @Override
    public void execute() {
        if (wasElevatorOpenAtStart) { return; }

        new SequentialCommandGroup(
            m_elevator.openElevatorCommand(),
            m_shooter.trapThrowCommand(),
            m_elevator.closeElevatorCommand()
        ).execute();
    }

    @Override
    public boolean isFinished() {
        if (wasElevatorOpenAtStart) { return true; }

        return false;
    }
}
