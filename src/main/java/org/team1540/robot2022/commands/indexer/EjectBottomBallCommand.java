package org.team1540.robot2022.commands.indexer;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.team1540.robot2022.commands.indexer.Indexer.IndexerState;
import org.team1540.robot2022.commands.intake.Intake;

public class EjectBottomBallCommand extends SequentialCommandGroup {
    public EjectBottomBallCommand(Indexer indexer, Intake intake) {
        addRequirements(indexer, intake);
        addCommands(
            parallel( // Stop indexer and shooter
                indexer.commandStop(),
                intake.commandStop()
            ),
            parallel(
                indexer.commandSet(IndexerState.UNCHANGED, IndexerState.REVERSE), // Run top of indexer
                intake.commandSetPercent(-0.5)
            ),
            new WaitUntilCommand(() -> !indexer.getBottomSensor()), // Wait until ball no longer seen by top sensor
            new WaitCommand(0.5), // Wait for ball to fully exit robot
            parallel( // Stop indexer and shooter again
                indexer.commandStop(),
                intake.commandStop()
            )
        );
    }


}
