package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class CloseClimberCommand extends SequentialCommandGroup{
    private final ShooterSubsystem m_shooter;
    private final ElevatorSubsystem m_elevator;

    public CloseClimberCommand (ShooterSubsystem shooterSubsystem, ElevatorSubsystem elevatorSubsystem) {
        m_shooter = shooterSubsystem;
        m_elevator = elevatorSubsystem;

        addRequirements(m_elevator);
        addRequirements(m_shooter);

        addCommands(
            /* Yeni şeyler eklenebilir */
            m_elevator.closeElevatorCommand()
        );
    }
}
