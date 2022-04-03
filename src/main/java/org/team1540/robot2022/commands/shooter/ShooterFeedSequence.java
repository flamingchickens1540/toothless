package org.team1540.robot2022.commands.shooter;

import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.utils.RevBlinkin;

public class ShooterFeedSequence extends SequentialCommandGroup {
    public ShooterFeedSequence(Indexer indexer, Shooter shooter, RevBlinkin lights, boolean has2Balls) {
        addRequirements(indexer);

        addCommands(
                new WaitUntilCommand(shooter::isSpunUp).withTimeout(1),                                             // Wait for shooter to spin up
                lights.commandSetPattern(RevBlinkin.ColorPattern.ORANGE),
                indexer.commandSet(Indexer.IndexerState.FORWARD_FULL, Indexer.IndexerState.OFF),     // Run top indexer
                new WaitCommand(1),
                new PrintCommand("topRPM " + indexer.topMotor.getSelectedSensorVelocity()),
                new WaitUntilCommand(() -> !indexer.getTopSensor()),                                 // Wait until top ball exits the indexer
                lights.commandSetPattern(RevBlinkin.ColorPattern.GREEN),
                // Shoot second ball
                new ConditionalCommand( // Only spend time shooting second ball if a second ball is staged
                        sequence(
                                indexer.commandSet(Indexer.IndexerState.FORWARD_FULL, Indexer.IndexerState.FORWARD), // Move bottom ball up
                                new ConditionalCommand(
                                        sequence(
                                                lights.commandSetPattern(RevBlinkin.ColorPattern.BLUE_GREEN),
                                                new WaitUntilCommand(indexer::getTopSensor),                                    // Wait until ball is in top staging location
                                                indexer.commandSet(Indexer.IndexerState.OFF, Indexer.IndexerState.OFF),         // Stop when ball is up high TODO: Standby here?
                                                new WaitUntilCommand(shooter::isSpunUp).withTimeout(0.5),
                                                indexer.commandSet(Indexer.IndexerState.FORWARD_FULL, Indexer.IndexerState.FORWARD_FULL), // Run both indexer wheels to shoot bottom ball
                                                new WaitCommand(1),
                                                new WaitUntilCommand(() -> !indexer.getTopSensor() && !indexer.getBottomSensor())      // Wait until ball leaves top of indexer
                                        ),
                                        new PrintCommand("hub shot, not waiting"),
                                        () -> true
                                )
                        ),
                        parallel(
                                indexer.commandSet(Indexer.IndexerState.OFF, Indexer.IndexerState.OFF),              // Stop the indexer
                                new PrintCommand("one ball, not shooting second")
                        ),
                        () -> indexer.getBottomSensor() || has2Balls
                )
        );
    }
}
