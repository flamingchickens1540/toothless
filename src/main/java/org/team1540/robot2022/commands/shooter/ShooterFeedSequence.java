package org.team1540.robot2022.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2022.commands.indexer.Indexer;

public class ShooterFeedSequence extends SequentialCommandGroup {
    public ShooterFeedSequence(Indexer indexer, Shooter shooter) {
        addRequirements(indexer);
        
        addCommands(
                new WaitUntilCommand(shooter::isSpunUp).withTimeout(1),                                             // Wait for shooter to spin up
                indexer.commandStop(),                                                               // Stop the indexer and put in standby
                indexer.commandSet(Indexer.IndexerState.FORWARD_FULL, Indexer.IndexerState.OFF),     // Run top indexer
                new WaitUntilCommand(() -> !indexer.getTopSensor()),                                 // Wait until top ball exits the indexer
                new WaitCommand(SmartDashboard.getNumber("shooter/tuning/waitAfterFirstBall", 0.5)), // Wait for top ball to leave and shooter to recover TODO: Can we reduce this?
                indexer.commandSet(Indexer.IndexerState.OFF, Indexer.IndexerState.OFF),              // Stop the indexer

                // Shoot second ball
                new ConditionalCommand( // Only spend time shooting second ball if a second ball is staged
                        sequence(
                                new WaitCommand(0.5),                                                           // TODO: Can we get rid of this?                                                      
                                indexer.commandSet(Indexer.IndexerState.FORWARD, Indexer.IndexerState.FORWARD), // Move bottom ball up
                                new WaitUntilCommand(indexer::getTopSensor),                                    // Wait until ball is in top staging location
                                indexer.commandSet(Indexer.IndexerState.OFF, Indexer.IndexerState.OFF),         // Stop when ball is up high TODO: Standby here?

                                new WaitUntilCommand(shooter::isSpunUp),                                                  // Wait until flywheel is spun up again
                                indexer.commandSet(Indexer.IndexerState.FORWARD_FULL, Indexer.IndexerState.FORWARD_FULL), // Run both indexer wheels to shoot bottom ball
                                new WaitUntilCommand(() -> !indexer.getTopSensor()),      // Wait until ball leaves top of indexer
                                new WaitCommand(0.5)                                      // Wait for bottom ball to be fully shot before stopping flywheels TODO: Can we reduce this?
                        ),
                        new InstantCommand(),
                        indexer::getBottomSensor
                )
        );
    }
}
