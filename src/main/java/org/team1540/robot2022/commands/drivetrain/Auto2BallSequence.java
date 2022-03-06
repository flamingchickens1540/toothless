package org.team1540.robot2022.commands.drivetrain;

import org.team1540.robot2022.commands.hood.Hood;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.commands.intake.Intake;
import org.team1540.robot2022.commands.shooter.Shooter;
import org.team1540.robot2022.commands.shooter.ShooterFeedSequence;
import org.team1540.robot2022.utils.AutoHelper;
import org.team1540.robot2022.utils.AutoSequence;
import org.team1540.robot2022.utils.AutoHelper.AutoPath;



public class Auto2BallSequence extends AutoSequence {

    public Auto2BallSequence(Drivetrain drivetrain, Intake intake, Indexer indexer, Shooter shooter, Hood hood, boolean isPosA) {
        AutoPath path = isPosA ? AutoPath.auto2Ball1A : AutoPath.auto2Ball1B;
        addPaths(path);
        
        addCommands(
                AutoHelper.runPathWithSpinup(drivetrain, intake, indexer, shooter, hood, path),  // Follow the path to collect the first ball
                new ShooterFeedSequence(indexer, shooter)
        );
    }
}
