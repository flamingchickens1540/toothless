package org.team1540.robot2022.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2022.commands.indexer.Indexer.IndexerState;
import org.team1540.robot2022.commands.intake.Intake;

public class IndexerEjectCommand extends CommandBase {
    private final Indexer indexer;
    private final Intake intake;

    public IndexerEjectCommand(Indexer indexer, Intake intake) {
        this.indexer = indexer;
        this.intake = intake;
        addRequirements(indexer, intake);
    }

    @Override
    public void initialize() {
        indexer.stop();
    }

    @Override
    public void execute() {
        indexer.set(IndexerState.REVERSE, IndexerState.REVERSE);
        intake.setPercent(-0.75);
    }

    @Override
    public void end(boolean isInterrupted) {
        indexer.stop();
        intake.stop();
    }
}
