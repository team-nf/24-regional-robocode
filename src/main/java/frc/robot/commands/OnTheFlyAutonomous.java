package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

// telefon githubdan yazıyorum importları sonradan hallederim kabataslak
// ikinci bir commit atıp squash yaparım sonra

/**
 * Create Autonomous Route on-the-fly using data from stereo-vision.
 * Find paths to closest notes. 
 */
public class OnTheFlyAutonomous extends Command{
    private final SwerveSubsystem driveBase;
    private final Supplier<double[]> visionDataSupplier;
    private final Supplier<Boolean> hasObject;

    private PathConstraints constraints;
    private double[] prevData;

    public OnTheFlyAutonomous(SwerveSubsystem driveBase, Supplier<double[]> visionDataSupplier, Supplier<Boolean> hasObject){
        this.driveBase = driveBase;
        constraints = this.driveBase.getConstraints();
        this.visionDataSupplier = visionDataSupplier;
        this.hasObject = hasObject;

        addRequirements(driveBase);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if (hasObject.get()) {
            // center drivebase, shoot note set hasObject to false

            // komutu bitirip andThen() ile diğer komutu çağırıp bunu döngüye almak iyi olabilir.
        } else {
        // döngüde pathfind yapmanın sonucu ne bilmiyorum
        // aşağıda koordinatların değişip değişmediğine bakan bir if koyucam bakalım

        // [ note_coordinate_x, note_coordinate_coordinate_y, note_angle_error, ...] şeklinde varsayıyorum
        double[] data = visionDataSupplier.get();
        if (prevData.length > 0 && prevData[0] == data[0] && prevData[1] == data[1]) {
            prevData = data;
        } else {
            AutoBuilder.pathfindToPose(
                new Pose2d(data[0], data[1], new Rotation2d(driveBase.getHeading().getRadians() - data[2])), 
                constraints
                );
        } 
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
