package org.team1540.robot2022.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeSpinCommand extends CommandBase {
    private final double speed;
    private final Intake intake;

    public IntakeSpinCommand(Intake intake, double speed) {
        this.intake = intake;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        intake.setPercent(this.speed);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPercent(0);
    }
}
