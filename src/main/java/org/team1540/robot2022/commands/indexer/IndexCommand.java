package org.team1540.robot2022.commands.indexer;

import org.team1540.robot2022.commands.indexer.Indexer.IndexerState;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class IndexCommand extends SequentialCommandGroup {
    private Indexer indexer;

    public IndexCommand(Indexer indexer) {
        this.indexer = indexer;
        addRequirements(indexer);
        addCommands(
                new ConditionalCommand(
                        new IndexReverseCommand(indexer,0.2,false),  // If (indexer is full)     -> stop indexer
                        parallel(               // If (indexer is not full) -> run enclosed
                                indexer.commandSetBottom(IndexerState.FORWARD), // Run bottom indexer
                                new ConditionalCommand(       
                                        new IndexReverseCommand(indexer,0.2,true),     // If (top sensor blocked)     -> Stop top indexer motor
                                        indexer.commandSetTop(IndexerState.FORWARD), // If not (top sensor blocked) -> Run top indexer motor
                                        indexer::getTopSensor         // Condition for top indexer motor
                                )
                        ),
                        indexer::isFull // Condition for indexer
                )
        );
    }

    @Override
    public void end(boolean isInterrupted) {
        this.indexer.stop();
    }
}

