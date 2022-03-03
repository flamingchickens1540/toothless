package org.team1540.robot2022.commands.indexer;

import org.team1540.robot2022.commands.indexer.Indexer.IndexerState;
import org.team1540.robot2022.commands.shooter.Shooter;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class EjectTopBallCommand extends SequentialCommandGroup {
    public EjectTopBallCommand(Indexer indexer, Shooter shooter) {
        addRequirements(indexer, shooter);
        addCommands(
            parallel( // Stop indexer and shooter
                indexer.commandStop(),
                shooter.commandStop()
            ),
            parallel(
                indexer.commandSet(IndexerState.FORWARD, IndexerState.UNCHANGED), // Run top of indexer
                shooter.commandSetVelocity(1000, 1000)                                 // Run shooter at low velocity
            ),
            new WaitUntilCommand(() -> !indexer.getTopSensor()), // Wait until ball no longer seen by top sensor
            new WaitCommand(0.2), // Wait for ball to fully exit robot
            parallel( // Stop indexer and shooter again
                indexer.commandStop(),
                shooter.commandStop()
            )
        );
    }
}
