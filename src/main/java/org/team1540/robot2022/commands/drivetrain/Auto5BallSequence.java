package org.team1540.robot2022.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import org.team1540.robot2022.commands.hood.Hood;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.commands.intake.Intake;
import org.team1540.robot2022.commands.shooter.Shooter;
import org.team1540.robot2022.commands.shooter.ShooterFeedSequence;
import org.team1540.robot2022.utils.AutoHelper;
import org.team1540.robot2022.utils.AutoSequence;
import org.team1540.robot2022.utils.AutoHelper.AutoPath;


public class Auto5BallSequence extends AutoSequence {
    public Auto5BallSequence(Drivetrain drivetrain, Intake intake, Indexer indexer, Shooter shooter, Hood hood) {
        addPaths(AutoPath.auto2Ball1A, AutoPath.auto5Ball2, AutoPath.auto5Ball3);
        addCommands(
                AutoHelper.runPathWithSpinup(drivetrain, intake, indexer, shooter, hood, AutoPath.auto2Ball1A), // Follow path to collect ball 2
                new ShooterFeedSequence(indexer, shooter),

                AutoHelper.runPathWithSpinup(drivetrain, intake, indexer, shooter, hood, AutoPath.auto5Ball2), // Follow path to collect balls 3 and 4
                new ConditionalCommand(                                                                         // If (2 balls are indexed) then shoot, else keep going. In case human player ball isn't present
                        new ShooterFeedSequence(indexer, shooter),
                        new PrintCommand("1 ball indexed; skipping shot"),
                        indexer::isFull
                ),

                AutoHelper.runPathWithSpinup(drivetrain, intake, indexer, shooter, hood, AutoPath.auto5Ball3), // Follow path to collect ball 5
                new ShooterFeedSequence(indexer, shooter)
        );
    }
}

