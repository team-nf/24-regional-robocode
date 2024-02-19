package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ClimbingModeCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final double climbingAngle;

    public ClimbingModeCommand(ShooterSubsystem shooterSubsystem, double climbingAngle) {
        this.shooterSubsystem = shooterSubsystem;
        this.climbingAngle = climbingAngle;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setShooterAngle(climbingAngle);
    }
}
