package org.team1540.robot2022.commands.indexer;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.team1540.robot2022.commands.indexer.Indexer.IndexerState;
import org.team1540.robot2022.commands.intake.Intake;

/**
 * Eject the bottom ball out of the intake
 */
public class EjectBottomBallCommand extends SequentialCommandGroup {
    private final Intake intake;
    private final Indexer indexer;
    
    public EjectBottomBallCommand(Indexer indexer, Intake intake) {
        this.indexer = indexer;
        this.intake = intake;

        addRequirements(indexer, intake);

        addCommands(
            parallel( // Stop indexer and shooter
                indexer.commandStop(),
                intake.commandStop()
            ),
            parallel(
                indexer.commandSet(IndexerState.UNCHANGED, IndexerState.REVERSE), // Reverse bottom indexer
                intake.commandSetPercent(-0.5)
            ),
            new WaitUntilCommand(() -> !indexer.getBottomSensor()), // Wait until ball no longer seen by top sensor
            new WaitCommand(0.5) // Wait for ball to fully exit robot
        );
    }

    @Override
    public void end(boolean isInterrupted) {
        indexer.stop();
        intake.stop();
    }
}
