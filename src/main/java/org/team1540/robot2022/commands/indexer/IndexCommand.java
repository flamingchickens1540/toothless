package org.team1540.robot2022.commands.indexer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class IndexCommand extends CommandBase {
    private Indexer indexer;
    private SequentialCommandGroup indexBottom, indexTop;

    public IndexCommand(Indexer indexer) {
        this.indexer = indexer;
        this.indexBottom = new IndexBottom(indexer).andThen(new InstantCommand(() -> {this.scheduleCommands();}));
        this.indexTop = new IndexTop(indexer).andThen(new InstantCommand(() -> {this.scheduleCommands();}));
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

    private void scheduleCommands() {
        if (this.indexToTop()) {
            indexTop.schedule();
        } else if (this.indexToBottom()) {
            indexBottom.schedule();
        }
    }
    @Override
    public void initialize() {
        DriverStation.reportWarning("Indexer Top "+indexToTop(), false);
        DriverStation.reportWarning("Indexer Bottom "+indexToBottom(), false);
        
        scheduleCommands();
    }

    @Override
    public void end(boolean isInterrupted) {
        indexTop.cancel();
        indexBottom.cancel();
    }
}
