package org.team1540.robot2022.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeTiltCommand extends CommandBase {
    private final boolean state;
    private final Intake intake;

    public IntakeTiltCommand(Intake intake, boolean state) {
        this.intake = intake;
        this.state = state;
    }

    @Override
    public void initialize() {
        intake.setTilt(this.state);
    }

    @Override
    public void execute() {
    }
}
