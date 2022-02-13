package org.team1540.robot2022.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IndexBottom extends CommandBase {
    private Indexer indexer;

    public IndexBottom(Indexer indexer) {
        this.indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void execute() {
        if (indexer.getBottomSensor()) {
            this.end(false);
        } else {
            indexer.set(false, true);
        }
    }
}
