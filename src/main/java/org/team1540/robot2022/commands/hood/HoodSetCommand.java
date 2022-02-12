package org.team1540.robot2022.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class HoodSetCommand extends CommandBase {
    private final boolean state;
    private final Hood hood;

    public HoodSetCommand(Hood hood, Boolean state) {
        this.hood = hood;
        this.state = state;
    }

    @Override
    public void initialize() {
        hood.set(this.state);
    }

    @Override
    public void execute() {
    }
}
