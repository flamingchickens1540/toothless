package org.team1540.robot2022.utils;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ChickenInstantCommand extends InstantCommand {
    private final boolean runsWhenDisabled;

    public ChickenInstantCommand(Runnable toRun, boolean runsWhenDisabled, Subsystem... requirements) {
        super(toRun, requirements);
        this.runsWhenDisabled = runsWhenDisabled;
    }

    @Override
    public boolean runsWhenDisabled() {
        return this.runsWhenDisabled;
    }
}
