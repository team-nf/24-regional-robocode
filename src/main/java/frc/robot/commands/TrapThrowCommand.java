package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class TrapThrowCommand extends Command {
    private final ShooterSubsystem m_shooter;
    private final ElevatorSubsystem m_elevator;
    private boolean wasElevatorClosedAtStart;
    private Command m_commandGroup;

    public TrapThrowCommand(double angle, ShooterSubsystem shooterSubsystem, ElevatorSubsystem elevatorSubsystem) {
        m_shooter = shooterSubsystem;
        m_elevator = elevatorSubsystem;

        addRequirements(m_elevator);
        addRequirements(m_shooter);

        /** Eğer elevator başlangıçta açıksa komudu çalıştırma */
        wasElevatorClosedAtStart = m_elevator.isElevatorFullyClosed();
    }

    @Override
    public void initialize() {
        if (!wasElevatorClosedAtStart) { return; }

        m_commandGroup = new SequentialCommandGroup(
            m_elevator.openElevatorCommand(),
            m_shooter.trapThrowCommand(),
            m_elevator.closeElevatorCommand()
        );
    }

    @Override
    public void execute() {
        if (!wasElevatorClosedAtStart) { return; }

        m_commandGroup.execute();
    }

    @Override
    public boolean isFinished() {
        if (!wasElevatorClosedAtStart) { return true; }

        return m_commandGroup.isFinished();
    }
}
