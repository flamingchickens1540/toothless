package org.team1540.robot2022.commands.indexer;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import org.team1540.robot2022.commands.indexer.Indexer.IndexerState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IndexReverseCommand extends CommandBase {
    private Timer timer = new Timer();
    private double duration;
    private Consumer<IndexerState> setterFunction;
    private BooleanSupplier shouldEnd;
    private boolean hasCompleted = false;

    /**
     * Constructs an IndexRollbackCommand that will run part of the indexer backwards for a duration from SmartDashboard, then stop
     * @param indexer The indexer subsystem
     * @param isTop If the operation should be performed on the top motor
     */
    public IndexReverseCommand(Indexer indexer, boolean isTop) {
        if (isTop) {
            this.setterFunction = indexer::setTop;
            this.shouldEnd = indexer::getTopSensor;
        } else {
            this.setterFunction = indexer::setBottom;
            this.shouldEnd = indexer::getBottomSensor;
        }
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        this.duration = 0.7;
        setterFunction.accept(IndexerState.REVERSE);
        hasCompleted = false;
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(duration) && !hasCompleted) {
            setterFunction.accept(IndexerState.OFF);
            hasCompleted = true;
        }
    }

    @Override
    public boolean isFinished() {
        return !this.shouldEnd.getAsBoolean();
    }

    @Override
    public void end(boolean isInterrupted) {
        setterFunction.accept(IndexerState.OFF);
        timer.stop();
    }
}
