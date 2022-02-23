package org.team1540.robot2022.commands.intake;

import org.team1540.robot2022.Constants;
import org.team1540.robot2022.commands.indexer.Indexer;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class IntakeSequence extends SequentialCommandGroup {
    private Intake intake;
    private Indexer indexer;

    public IntakeSequence(Intake intake, Indexer indexer) {
        this.intake = intake;
        this.indexer = indexer;
        addRequirements(intake, indexer);
        addCommands(
                indexer.commandStart(),
                new IntakeSpinCommand(intake, indexer, Constants.IntakeConstants.speed)
                
        );
    }


    @Override
    public void end(boolean isInterrupted) {
        intake.stop();
        indexer.stop();
    }
}
