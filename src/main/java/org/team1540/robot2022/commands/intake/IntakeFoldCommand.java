package org.team1540.robot2022.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeFoldCommand extends CommandBase {
    private final boolean state;
    private final Intake intake;

    public IntakeFoldCommand(Intake intake, boolean state) {
        this.intake = intake;
        this.state = state;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setFold(this.state);
    }

    @Override
    public void execute() {
    }
}
