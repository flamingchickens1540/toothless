package org.team1540.robot2022.commands.drivetrain;

import org.team1540.robot2022.commands.hood.Hood;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.commands.intake.Intake;
import org.team1540.robot2022.commands.shooter.Shooter;
import org.team1540.robot2022.commands.shooter.ShooterFeedSequence;
import org.team1540.robot2022.utils.AutoHelper;
import org.team1540.robot2022.utils.AutoSequence;
import org.team1540.robot2022.utils.AutoHelper.AutoPath;


public class Auto4BallSequence extends AutoSequence {
    public Auto4BallSequence(Drivetrain drivetrain, Intake intake, Indexer indexer, Shooter shooter, Hood hood) {
        addPaths(AutoPath.auto2Ball1A, AutoPath.auto4Ball2);
        addCommands(
            AutoHelper.runPathWithSpinup(drivetrain, intake, indexer, shooter, hood, AutoPath.auto2Ball1A), // Follow path to collect ball 1 and spinup flywheels
            new ShooterFeedSequence(indexer, shooter),                                                      // Feed 2 balls into shooter

            AutoHelper.runPathWithSpinup(drivetrain, intake, indexer, shooter, hood, AutoPath.auto4Ball2), // Follow path to collect balls 3 and 4 and spinup flywheels
            new ShooterFeedSequence(indexer, shooter)                                                       // Feed the 2 balls into the shooter
        );
    }
}

