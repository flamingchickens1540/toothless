package org.team1540.robot2022.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2022.commands.indexer.Indexer;

import java.util.function.Supplier;

public class IntakeSpinCommand extends CommandBase {
    private final double speed;
    private final Intake intake;
    private final Supplier<Boolean> finishSupplier;

    public IntakeSpinCommand(Intake intake, Indexer indexer, double speed) {
        this.intake = intake;
        this.finishSupplier = indexer::isFull;
        this.speed = speed;
        addRequirements(intake);
    }

    public IntakeSpinCommand(Intake intake, double speed) {
        this.intake = intake;
        this.finishSupplier = () -> {return false;};
        this.speed = speed;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (intake.getFold()) {
            intake.setFold(false);
        }
        intake.setPercent(this.speed);
    }

    @Override
    public void execute() {
    }

    public boolean isFinished() {
        return finishSupplier.get();
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPercent(0);
    }
}
