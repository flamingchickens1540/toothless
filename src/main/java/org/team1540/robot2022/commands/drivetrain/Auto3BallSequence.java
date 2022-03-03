package org.team1540.robot2022.commands.drivetrain;

import org.team1540.robot2022.commands.hood.Hood;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.commands.intake.Intake;
import org.team1540.robot2022.commands.shooter.ShootSequence;
import org.team1540.robot2022.commands.shooter.Shooter;
import org.team1540.robot2022.utils.AutoHelper;
import org.team1540.robot2022.utils.LIDAR;
import org.team1540.robot2022.utils.Limelight;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Auto3BallSequence extends SequentialCommandGroup {

    public Auto3BallSequence(Drivetrain drivetrain, Intake intake, Indexer indexer, Shooter shooter, Hood hood, Limelight limelight, LIDAR lidar) {
        addCommands(
                AutoHelper.runPath(drivetrain, intake, indexer, shooter,"2ball.posA.path1.wpilib.json"), // Follow path to collect first ball
                new ShootSequence(shooter, indexer, drivetrain, hood, intake, limelight, lidar, false),                       // Shoot the 2 indexed balls (starts with one, collects one)
                AutoHelper.runPath(drivetrain, intake, indexer, shooter,"3ball.posA.path2.wpilib.json"), // Follow path to collect second ball
                new ShootSequence(shooter, indexer, drivetrain, hood, intake, limelight, lidar, false)                        // Shoot the 2 indexed balls (starts with one, collects one)
        );
    }
}
