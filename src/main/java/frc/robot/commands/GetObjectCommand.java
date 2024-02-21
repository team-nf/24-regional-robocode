package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class GetObjectCommand extends Command {
    private final ShooterSubsystem m_shooter;
    private final IntakeSubsystem m_intake;

    public GetObjectCommand(double angle, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        m_intake = intakeSubsystem;
        m_shooter = shooterSubsystem;

        addRequirements(shooterSubsystem);
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        /* Eğer shooterda obje varsa bu fonksiyonlar çalışmamalı*/
        if (m_shooter.hasObject()) { return; }

        m_intake.runIntakeCommand();
        m_shooter.runFeederCommand();
    }

    @Override
    public void execute() {
        /** Obje gelene kadar bekle */
        while (!m_shooter.hasObject()) {}
        /* Gelince motorları durdur */
        m_intake.stopIntakeCommand();
        m_shooter.stopFeederCommand();
    }

    @Override
    public boolean isFinished() {
        return m_shooter.hasObject();
    }
}
