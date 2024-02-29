package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class GetObjectCommand extends Command {
    private final ShooterSubsystem m_shooter;
    private final IntakeSubsystem m_intake;
    private Command m_runMotorsCommand, m_stopMotorsCommand;
    private boolean m_shooterGotObject = false;

    public GetObjectCommand(double angle, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
        m_intake = intakeSubsystem;
        m_shooter = shooterSubsystem;

        addRequirements(m_intake);
        addRequirements(m_shooter);

        m_shooterGotObject = m_shooter.hasObject();
    }

    @Override
    public void initialize() {
        /* Eğer shooterda obje varsa bu fonksiyonlar çalışmamalı*/
        if (m_shooterGotObject) { return; }

        m_runMotorsCommand = new SequentialCommandGroup(
            m_intake.runIntakeCommand(),
            m_shooter.runFeederCommand()
        );

        m_stopMotorsCommand = new SequentialCommandGroup(
            m_intake.stopIntakeCommand(),
            m_shooter.stopFeederCommand()
        );
    }

    @Override
    public void execute() {
        /* Öncelikle motorları çalıştır ve hızlanmasını bekle*/
        if (!m_runMotorsCommand.isFinished() && m_shooterGotObject == false) {
            m_runMotorsCommand.execute();
            return;
        }

        /*
         * Önceden shooterda obje görüldüyse hasObject
         * fonksiyonunu baştan çağırmamak için 2 if var,
         * çünkü sensör okumada delay olabilir
        */
        if (m_shooterGotObject == false) {
            if (m_shooter.hasObject())
                m_shooterGotObject = true;
        }

        if (m_shooterGotObject) {
            m_stopMotorsCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        return m_shooterGotObject && m_stopMotorsCommand.isFinished();
    }
}
