package org.team1540.robot2022.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.team1540.robot2022.commands.hood.Hood;
import org.team1540.robot2022.commands.indexer.EjectTopBallCommand;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.commands.intake.Intake;
import org.team1540.robot2022.commands.shooter.ShootSequence;
import org.team1540.robot2022.commands.shooter.Shooter;
import org.team1540.robot2022.commands.vision.Vision;
import org.team1540.robot2022.utils.*;
import org.team1540.robot2022.utils.AutoHelper.AutoPath;

/**
 * This two ball will additionally collect an opponent ball and move it away from the center of the field into our hangar
 * <p>
 * Graciously and professionally, of course
 */
public class Auto2BallMeanSequence extends AutoSequence {
    public Auto2BallMeanSequence(Drivetrain drivetrain, Intake intake, Indexer indexer, Vision vision, Shooter shooter, Hood hood, Limelight limelight, LIDAR lidar, NavX navx) {
        addPaths(AutoPath.auto2Ball1B, AutoPath.auto2Ball2BM);
        addCommands(
                AutoHelper.runPathWithSpinup(drivetrain, intake, indexer, shooter, hood, AutoPath.auto2Ball1B),  // Follow the path to collect the first ball
                new ShootSequence(shooter,
                        indexer,
                        drivetrain,
                        hood,
                        intake,
                        vision,
                        limelight,
                        lidar,
                        navx,
                        Shooter.ShooterProfile.FAR,
                        true, true, null),
                AutoHelper.runPath(drivetrain, intake, indexer, shooter, AutoPath.auto2Ball1B),
                new WaitUntilCommand(indexer::getTopSensor),
                new EjectTopBallCommand(indexer, shooter)
        );
    }
}

