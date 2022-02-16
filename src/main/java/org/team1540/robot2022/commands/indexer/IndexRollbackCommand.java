package org.team1540.robot2022.commands.indexer;

import java.util.function.Consumer;

import org.team1540.robot2022.commands.indexer.Indexer.IndexerState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IndexRollbackCommand extends CommandBase {
    private Timer timer = new Timer();
    private double duration;
    private Consumer<IndexerState> setterFunction;

    /**
     * Constructs an IndexRollbackCommand that will run part of the indexer backwards for a specified duration, then stop
     * @param indexer The indexer subsystem
     * @param duration The duration to run in reverse for
     * @param isTop If the operation should be performed on the top motor
     */
    public IndexRollbackCommand(Indexer indexer, double duration, boolean isTop) {
        this.setterFunction = isTop ? indexer::setTop : indexer::setBottom; // Store the function to set motors with based on the value of isTop
        this.duration = duration;
        addRequirements(indexer);
    }

    /**
     * Constructs an IndexRollbackCommand that will run part of the indexer backwards for a duration from SmartDashboard, then stop
     * @param indexer The indexer subsystem
     * @param isTop If the operation should be performed on the top motor
     */
    public IndexRollbackCommand(Indexer indexer, boolean isTop) {
        if (isTop) {
            this.setterFunction = indexer::setTop;
            this.duration = SmartDashboard.getNumber("indexer/waitDuration/top", 0.2);
        } else {
            this.setterFunction = indexer::setBottom;
            this.duration = SmartDashboard.getNumber("indexer/waitDuration/bottom", 0.2);
        }
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        setterFunction.accept(IndexerState.REVERSE);
        
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(duration)) {
            setterFunction.accept(IndexerState.OFF);
        }
    }

    @Override
    public void end(boolean isInterrupted) {
        timer.stop();
    }
}
