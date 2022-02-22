package org.team1540.robot2022.commands.drivetrain;

import org.team1540.robot2022.commands.hood.Hood;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.commands.intake.Intake;
import org.team1540.robot2022.commands.shooter.ShootSequence;
import org.team1540.robot2022.commands.shooter.Shooter;
import org.team1540.robot2022.utils.AutoSequenceWrapper;
import org.team1540.robot2022.utils.Limelight;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Auto3BallSequence extends SequentialCommandGroup {

    public Auto3BallSequence(Drivetrain drivetrain, Intake intake, Indexer indexer, Shooter shooter, Hood hood, Limelight limelight) {
        addCommands(
                // TODO: These null commands need fixing
                AutoSequenceWrapper.runPath(drivetrain, intake, indexer, "2ball.posA.path1.wpilib.json"), // Follow path to collect first ball
                new ShootSequence(shooter, indexer, drivetrain, hood, intake, limelight, null),                       // Shoot the 2 indexed balls (starts with one, collects one)
                AutoSequenceWrapper.runPath(drivetrain, intake, indexer, "3ball.posA.path2.wpilib.json"), // Follow path to collect second ball
                new ShootSequence(shooter, indexer, drivetrain, hood, intake, limelight, null)                        // Shoot the 2 indexed balls (starts with one, collects one)
        );
    }
}
