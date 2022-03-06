package org.team1540.robot2022.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2022.commands.indexer.Indexer;

public class IntakeSpinCommand extends CommandBase {
    private final double speed;
    private final Intake intake;
    private final Indexer indexer;

    public IntakeSpinCommand(Intake intake, Indexer indexer, double speed) {
        this.intake = intake;
        this.indexer = indexer;
        this.speed = speed;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setFold(false);
        intake.setPercent(this.speed);
    }

    @Override
    public void execute() {
    }

    public boolean isFinished() {
        return indexer.isFull();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
