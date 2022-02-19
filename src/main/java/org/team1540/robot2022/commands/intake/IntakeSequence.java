package org.team1540.robot2022.commands.intake;

import org.team1540.robot2022.commands.indexer.IndexCommand;
import org.team1540.robot2022.commands.indexer.Indexer;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class IntakeSequence extends SequentialCommandGroup{
    private Intake intake;
    private Indexer indexer;
    public IntakeSequence(Intake intake, Indexer indexer) {
        this.intake = intake;
        this.indexer = indexer;
        addRequirements(intake, indexer);
        addCommands(
            new ConditionalCommand( 
                parallel( // if (indexer is full) -> Stop intake, wait, stop indexer
                    intake.commandStop(), // stop intake
                    sequence(             
                        new WaitCommand(1),   // wait
                        indexer.commandStop() // stop indexer
                    )
                ),
                parallel( // if (indexer is not full) -> run intake and indexer
                    new IndexCommand(indexer, intake),         // run indexer
                    new IntakeSpinCommand(intake, 0.5) // run intake
                ),
                indexer::isFull
            )
        );
    }

    public void end(boolean isInterrupted) {
        intake.stop();
        indexer.stop();
    }
}
