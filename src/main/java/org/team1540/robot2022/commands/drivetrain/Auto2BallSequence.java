package org.team1540.robot2022.commands.drivetrain;

import org.team1540.robot2022.commands.hood.Hood;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.commands.intake.Intake;
import org.team1540.robot2022.commands.shooter.ShootSequence;
import org.team1540.robot2022.commands.shooter.Shooter;
import org.team1540.robot2022.utils.*;
import org.team1540.robot2022.utils.AutoHelper.AutoPath;


public class Auto2BallSequence extends AutoSequence {

    public Auto2BallSequence(Drivetrain drivetrain, Intake intake, Indexer indexer, Shooter shooter, Hood hood, Limelight limelight, LIDAR lidar, NavX navx, boolean isPosA) {
        AutoPath path = isPosA ? AutoPath.auto2Ball1A : AutoPath.auto2Ball1B;
        addPaths(path);

        addCommands(
                AutoHelper.runPathWithSpinup(drivetrain, intake, indexer, shooter, hood, path),  // Follow the path to collect the first ball

                new ShootSequence(shooter,
                        indexer,
                        drivetrain,
                        hood,
                        intake,
                        limelight,
                        lidar,
                        navx,
                        Shooter.ShooterProfile.FAR,
                        !isPosA)
        );
    }
}
