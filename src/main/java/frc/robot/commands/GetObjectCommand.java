package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class GetObjectCommand extends Command {
    private final ShooterSubsystem m_shooter;
    private final IntakeSubsystem m_intake;
    // private Command m_runMotorsCommand, m_stopMotorsCommand;
    private boolean m_shooterGotObject = false;
    private boolean runCommandsInitialized = false, stopCommandsInitialized = false;

    private Command runIntakeCommand, runFeederCommand, stopIntakeCommand, stopFeederCommand;


    public GetObjectCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
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

        runIntakeCommand = m_intake.runIntakeCommand();
        runFeederCommand = m_shooter.runFeederCommand();

        stopIntakeCommand = m_intake.stopIntakeCommand();
        stopFeederCommand = m_shooter.stopFeederCommand();
    }

    @Override
    public void execute() {
        /* Obje yoksa */
        if (m_shooterGotObject == false) {
            if (m_shooter.hasObject())
                m_shooterGotObject = true;

            /* Öncelikle motorları çalıştır ve hızlanmasını bekle*/
            if (!runIntakeCommand.isFinished() || !runFeederCommand.isFinished()) {
                if (!runCommandsInitialized) {
                    runIntakeCommand.initialize();
                    runFeederCommand.initialize();

                    runCommandsInitialized = true;
                }

                runIntakeCommand.execute();
                runFeederCommand.execute();
                return;
            }
        }

        if (m_shooterGotObject) {
            if (stopCommandsInitialized == false) {
                stopIntakeCommand.initialize();
                stopFeederCommand.initialize();

                stopCommandsInitialized = true;
            }
            stopIntakeCommand.execute();
            stopFeederCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        return m_shooterGotObject;
    }
}
