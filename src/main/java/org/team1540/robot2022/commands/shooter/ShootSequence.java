package org.team1540.robot2022.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2022.commands.drivetrain.Drivetrain;
import org.team1540.robot2022.commands.drivetrain.PointToTarget;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.utils.Limelight;

public class ShootSequence extends SequentialCommandGroup {
    private final Command indexCommand;
    private boolean indexCommandScheduled;

    private final Shooter shooter;
    private final Indexer indexer;

    public ShootSequence(Shooter shooter, Indexer indexer, Drivetrain drivetrain, Limelight limelight, Command indexCommand) {
        this.indexCommand = indexCommand;
        this.shooter = shooter;
        this.indexer = indexer;
        addRequirements(shooter, indexer, drivetrain);
        addCommands(
                sequence(
                        new InstantCommand(() -> {
                            indexCommandScheduled = indexCommand.isScheduled();
                            indexCommand.cancel();
                        }),
                        parallel(
                            new InstantCommand(() -> {
                                shooter.setVelocityRPM(shooter.shooterMotorFront, SmartDashboard.getNumber("shooter/tuning/frontRPM", 0));
                                shooter.setVelocityRPM(shooter.shooterMotorRear, SmartDashboard.getNumber("shooter/tuning/rearRPM", 0));
                            }, shooter),
                            new PointToTarget(drivetrain, limelight)
                        ),
                        new WaitUntilCommand(shooter::isSpunUp),
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
