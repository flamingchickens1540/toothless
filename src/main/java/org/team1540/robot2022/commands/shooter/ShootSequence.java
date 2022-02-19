package org.team1540.robot2022.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team1540.robot2022.commands.indexer.Indexer;

public class ShootSequence extends SequentialCommandGroup {
    private final Command indexCommand;
    private boolean indexCommandScheduled;

    private final Shooter shooter;
    private final Indexer indexer;

    public ShootSequence(Shooter shooter, Indexer indexer, Command indexCommand) {
        this.indexCommand = indexCommand;
        this.shooter = shooter;
        this.indexer = indexer;
        addRequirements(shooter, indexer);
        addCommands(
                sequence(
                        new InstantCommand(() -> {
                            indexCommandScheduled = indexCommand.isScheduled();
                            indexCommand.cancel();
                        }),

                        new InstantCommand(() -> {
                            shooter.setVelocityRPM(shooter.shooterMotorFront, -5000);
                            shooter.setVelocityRPM(shooter.shooterMotorRear, -5000);
                        }, shooter),
                        new WaitCommand(2), // TODO: Wait for shooter to reach velocity
                        new InstantCommand(() -> indexer.set(Indexer.IndexerState.FORWARD_FULL, Indexer.IndexerState.FORWARD_FULL)),
                        new WaitCommand(2),
                        new InstantCommand(() -> {
                            indexer.set(Indexer.IndexerState.OFF, Indexer.IndexerState.OFF);
                            shooter.setVelocityRPM(shooter.shooterMotorFront, 0);
                            shooter.setVelocityRPM(shooter.shooterMotorRear, 0);
                        })
                )
        );
    }

    @Override
    public void end(boolean isInterrupted) {
        shooter.stop();
        indexer.stop();

        if (indexCommandScheduled) {
            indexCommand.schedule();
        }
    }
}
