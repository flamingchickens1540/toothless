package org.team1540.robot2022.commands.shooter;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.utils.RevBlinkin;

public class ShooterFeedSequence extends SequentialCommandGroup {
    public ShooterFeedSequence(Indexer indexer, Shooter shooter, RevBlinkin lights) {
        addRequirements(indexer);

        addCommands(
                lights.commandSetPattern(RevBlinkin.ColorPattern.RED),
                new WaitUntilCommand(shooter::isSpunUp).withTimeout(1),                                             // Wait for shooter to spin up
                lights.commandSetPattern(RevBlinkin.ColorPattern.YELLOW),
                indexer.commandSet(Indexer.IndexerState.FORWARD_FULL, Indexer.IndexerState.OFF),     // Run top indexer
                new WaitUntilCommand(() -> !indexer.getTopSensor()),                                 // Wait until top ball exits the indexer
                lights.commandSetPattern(RevBlinkin.ColorPattern.GREEN),
                // Shoot second ball
                new ConditionalCommand( // Only spend time shooting second ball if a second ball is staged
                        sequence(
                                indexer.commandSet(Indexer.IndexerState.FORWARD, Indexer.IndexerState.FORWARD), // Move bottom ball up
                                lights.commandSetPattern(RevBlinkin.ColorPattern.BLUE_GREEN),
                                new WaitUntilCommand(indexer::getTopSensor),                                    // Wait until ball is in top staging location
                                lights.commandSetPattern(RevBlinkin.ColorPattern.BLUE),
                                indexer.commandSet(Indexer.IndexerState.OFF, Indexer.IndexerState.OFF),         // Stop when ball is up high TODO: Standby here?

                                new WaitUntilCommand(shooter::isSpunUp).withTimeout(0.5),
                                lights.commandSetPattern(RevBlinkin.ColorPattern.ORANGE),// Wait until flywheel is spun up again
                                indexer.commandSet(Indexer.IndexerState.FORWARD_FULL, Indexer.IndexerState.FORWARD_FULL), // Run both indexer wheels to shoot bottom ball
                                new WaitUntilCommand(() -> !indexer.getTopSensor())      // Wait until ball leaves top of indexer
                        ),
                        indexer.commandSet(Indexer.IndexerState.OFF, Indexer.IndexerState.OFF),              // Stop the indexer
                        indexer::getBottomSensor
                )
        );
    }
}
