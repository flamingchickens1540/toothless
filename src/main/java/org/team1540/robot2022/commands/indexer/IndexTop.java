package org.team1540.robot2022.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IndexTop extends CommandBase {
    private Indexer indexer;

    public IndexTop(Indexer indexer) {
        this.indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void execute() {
        if (indexer.getTopSensor()) {
            this.end(false);
        } else {
            indexer.set(true, true);
        }
    }
}
