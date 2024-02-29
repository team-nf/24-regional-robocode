package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class ClimbCommand extends Command {
    private final ShooterSubsystem m_shooter;
    private final IntakeSubsystem m_intake;

    public ClimbCommand(double angle, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        m_intake = intakeSubsystem;
        m_shooter = shooterSubsystem;

        addRequirements(m_intake);
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        /* Eğer shooterda obje varsa bu fonksiyonlar çalışmamalı*/
        if (m_shooter.hasObject()) { return; }

        new SequentialCommandGroup(
            m_intake.runIntakeCommand(),
            m_shooter.runFeederCommand()
        ).execute();
    }

    @Override
    public void execute() {
        /** Obje gelene kadar bekle */
        while (!m_shooter.hasObject()) {}
        /* Gelince motorları durdur */
        new SequentialCommandGroup(
            m_intake.stopIntakeCommand(),
            m_shooter.stopFeederCommand()
        ).execute();
    }

    @Override
    public boolean isFinished() {
        return m_shooter.hasObject();
    }
}
