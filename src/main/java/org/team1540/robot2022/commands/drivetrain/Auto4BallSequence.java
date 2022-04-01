package org.team1540.robot2022.commands.drivetrain;

import org.team1540.robot2022.commands.hood.Hood;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.commands.intake.Intake;
import org.team1540.robot2022.commands.shooter.ShootSequence;
import org.team1540.robot2022.commands.shooter.Shooter;
import org.team1540.robot2022.commands.vision.Vision;
import org.team1540.robot2022.utils.*;
import org.team1540.robot2022.utils.AutoHelper.AutoPath;


public class Auto4BallSequence extends AutoSequence {
    public Auto4BallSequence(Drivetrain drivetrain, Intake intake, Indexer indexer, Vision vision, Shooter shooter, Hood hood, Limelight limelight, LIDAR lidar, NavX navx) {
        addPaths(AutoPath.auto2Ball1A, AutoPath.auto4Ball2);
        addCommands(
                AutoHelper.runPathWithSpinup(drivetrain, intake, indexer, shooter, hood, AutoPath.auto2Ball1A), // Follow path to collect ball 1 and spinup flywheels
                new ShootSequence(shooter, indexer, drivetrain, hood, intake, vision, limelight, lidar, navx, Shooter.ShooterProfile.FAR, false),

                AutoHelper.runPathWithSpinup(drivetrain, intake, indexer, shooter, hood, AutoPath.auto4Ball2),   // Follow path to collect balls 3 and 4 and spinup flywheels
                new ShootSequence(shooter, indexer, drivetrain, hood, intake, vision, limelight, lidar, navx, Shooter.ShooterProfile.FAR, false)
        );
    }
}

