package org.team1540.robot2022.commands.drivetrain;

import org.team1540.robot2022.commands.hood.Hood;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.commands.intake.Intake;
import org.team1540.robot2022.commands.shooter.ShootSequence;
import org.team1540.robot2022.commands.shooter.Shooter;
import org.team1540.robot2022.utils.AutoHelper;
import org.team1540.robot2022.utils.LIDAR;
import org.team1540.robot2022.utils.Limelight;
import org.team1540.robot2022.utils.NavX;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Auto5BallSequence extends SequentialCommandGroup {
    public Auto5BallSequence(Drivetrain drivetrain, Intake intake, Indexer indexer, Shooter shooter, Hood hood, Limelight limelight, LIDAR lidar, NavX navx) {
        addCommands(
                AutoHelper.runPath(drivetrain, intake, indexer, shooter, "2ball.posA.path1.wpilib.json"), // Path follow to collect second ball
                new ShootSequence(shooter, indexer, drivetrain, hood, intake, limelight, lidar, navx, false),        // Shoot the 2 indexed balls (starts with one, collects one)
                AutoHelper.runPath(drivetrain, intake, indexer, shooter, "5ball.posA.path2.wpilib.json"), // Path follow to collect third and fourth balls
                new ConditionalCommand(                                                          // If (2 balls are indexed) then shoot, else keep going
                    new ShootSequence(shooter, indexer, drivetrain, hood, intake, limelight, lidar, navx, false), 
                    new PrintCommand("1 ball indexed; skipping shot"), 
                    indexer::isFull
                ),
                AutoHelper.runPath(drivetrain, intake, indexer, shooter, "5ball.posA.path3.wpilib.json"), // Path follow to collect fifth ball
                new ShootSequence(shooter, indexer, drivetrain, hood, intake, limelight, lidar, navx, false)         // Shoot the fifth ball
        );
    }
}

