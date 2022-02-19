package org.team1540.robot2022.commands.indexer;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2022.commands.indexer.Indexer.IndexerState;
import org.team1540.robot2022.commands.intake.Intake;

public class IndexCommand extends SequentialCommandGroup {
    private final Indexer indexer;
    private final Intake intake;

    public IndexCommand(Indexer indexer, Intake intake) {
        this.indexer = indexer;
        this.intake = intake;
        addRequirements(indexer, intake);
        addCommands(
                new ConditionalCommand(
                        parallel( // if (indexer is full)     -> stop indexer
                                indexer.commandStop(),
                                intake.commandStop()
                        ),
                        parallel(               // If (indexer is not full) -> run enclosed
                                new InstantCommand(() -> intake.setPercent(0.5)),
                                indexer.commandSetBottom(IndexerState.FORWARD), // Run bottom indexer
                                new ConditionalCommand(
                                        indexer.commandSetTop(IndexerState.OFF),     // If (top sensor blocked)     -> Stop top indexer motor
                                        indexer.commandSetTop(IndexerState.FORWARD), // If not (top sensor blocked) -> Run top indexer motor
                                        indexer::getTopSensor                        // Condition for top indexer motor
                                )
                        ),
                        indexer::isFull // Condition for indexer
                )
        );
    }

    @Override
    public void end(boolean isInterrupted) {
        if (isInterrupted) {
            this.indexer.stop();
            this.intake.stop();
        }
    }
}
