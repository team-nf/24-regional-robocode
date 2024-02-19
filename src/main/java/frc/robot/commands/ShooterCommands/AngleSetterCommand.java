package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class AngleSetterCommand extends Command {
    private final double angle;
    private final ShooterSubsystem shooterSubsystem;

    public AngleSetterCommand(double angle, ShooterSubsystem shooterSubsystem) {
        this.angle = angle;
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setShooterAngle(angle);
    }
}
