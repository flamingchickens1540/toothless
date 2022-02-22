package org.team1540.robot2022.commands.drivetrain;

import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.commands.intake.Intake;
import org.team1540.robot2022.commands.shooter.ShootSequence;
import org.team1540.robot2022.commands.shooter.Shooter;
import org.team1540.robot2022.utils.AutoSequenceWrapper;
import org.team1540.robot2022.utils.Limelight;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Auto4BallSequence extends SequentialCommandGroup {

    public Auto4BallSequence(Drivetrain drivetrain, Intake intake, Indexer indexer, Shooter shooter, Limelight limelight) {
        addCommands(
                AutoSequenceWrapper.runPath(drivetrain, intake, indexer, "2ball.posA.path1.wpilib.json"), // Path follow to collect first ball
                new ShootSequence(shooter, indexer, drivetrain, intake, limelight),                       // Shoot the 2 indexed balls (starts with one, collects one)
                AutoSequenceWrapper.runPath(drivetrain, intake, indexer, "4ball.posA.path2.wpilib.json"), // Path follow to collect second ball
                new ShootSequence(shooter, indexer, drivetrain, intake, limelight)                        // Shoot the 2 indexed balls (starts with one, collects one)
        );
    }
}

