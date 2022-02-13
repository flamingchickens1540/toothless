package org.team1540.robot2022.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class IndexCommand extends CommandBase {
    private Indexer indexer;
    private IndexBottom indexBottom;
    private IndexTop indexTop;

    public IndexCommand(Indexer indexer) {
        this.indexer = indexer;
        this.indexBottom = new IndexBottom(indexer);
        this.indexTop = new IndexTop(indexer);
        addRequirements(indexer);
    }

    private boolean indexToTop() {
        return (!indexer.getBottomSensor() && !indexer.getTopSensor());
    }
    private boolean indexToBottom() {
        return (!indexer.getBottomSensor() && indexer.getTopSensor());
    }

    private boolean indexerFull() {
        return (indexer.getBottomSensor() && indexer.getTopSensor());
    }

    @Override
    public void initialize() {
        new Trigger(this::indexToTop)
                .whileActiveOnce(indexTop);
        new Trigger(this::indexToBottom)
                .whileActiveOnce(indexBottom);
    }
}
