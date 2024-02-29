package frc.robot.commands.ClimbCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class OpenClimberCommand extends SequentialCommandGroup {
    private final ShooterSubsystem m_shooter;
    private final ElevatorSubsystem m_elevator;

    public OpenClimberCommand(ShooterSubsystem shooterSubsystem, ElevatorSubsystem elevatorSubsystem) {
        m_shooter = shooterSubsystem;
        m_elevator = elevatorSubsystem;

        addRequirements(m_elevator);
        addRequirements(m_shooter);

        addCommands(
            m_shooter.setClimbingAngleCommand(),
            m_elevator.openElevatorCommand()
        );
    }
}