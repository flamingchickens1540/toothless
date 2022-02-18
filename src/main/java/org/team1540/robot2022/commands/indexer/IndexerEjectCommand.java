package org.team1540.robot2022.commands.indexer;

import org.team1540.robot2022.commands.indexer.Indexer.IndexerState;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IndexerEjectCommand extends CommandBase {
    private Indexer indexer;


    public IndexerEjectCommand(Indexer indexer) {
        this.indexer = indexer;

    }

    @Override
    public void initialize() {
        System.out.println("Intialized");
        
    }

    @Override
    public void execute() {
        indexer.set(IndexerState.REVERSE, IndexerState.REVERSE);
    }

    @Override
    public void end(boolean isInterrputed) {
        System.out.println("IndexEjectEnd");
        indexer.set(IndexerState.OFF, IndexerState.OFF);
        
    }
    
}
