package org.team1540.robot2022.commands.drivetrain;

import org.team1540.robot2022.commands.hood.Hood;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.commands.intake.Intake;
import org.team1540.robot2022.commands.shooter.ShootSequence;
import org.team1540.robot2022.commands.shooter.Shooter;
import org.team1540.robot2022.commands.vision.Vision;
import org.team1540.robot2022.utils.*;
import org.team1540.robot2022.utils.AutoHelper.AutoPath;

public class Auto3BallSequence extends AutoSequence {
    public Auto3BallSequence(Drivetrain drivetrain, Intake intake, Indexer indexer, Vision vision, Shooter shooter, Hood hood, Limelight limelight, LIDAR lidar, NavX navx) {
        addPaths(AutoPath.auto2Ball1A, AutoPath.auto3Ball2);
        addCommands(
                AutoHelper.runPathWithSpinup(drivetrain, intake, indexer, shooter, hood, AutoPath.auto2Ball1A), // Follow path to collect first ball
                new ShootSequence(shooter, indexer, drivetrain, hood, intake, vision, limelight, lidar, navx, Shooter.ShooterProfile.FAR, false, true),

                AutoHelper.runPathWithSpinup(drivetrain, intake, indexer, shooter, hood, AutoPath.auto3Ball2), // Follow path to collect second ball
                new ShootSequence(shooter, indexer, drivetrain, hood, intake, vision, limelight, lidar, navx, Shooter.ShooterProfile.FAR, false, false)
        );
    }
}
