package org.team1540.robot2022.commands.shooter;

import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2022.InterpolationTable;
import org.team1540.robot2022.commands.drivetrain.Drivetrain;
import org.team1540.robot2022.commands.drivetrain.PointToTarget;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.utils.Limelight;

public class ShootSequence extends SequentialCommandGroup {
    private final Command indexCommand;
    private final Shooter shooter;
    private final Indexer indexer;
    private final Limelight limelight;
    private final InterpolationTable interpolationTable = InterpolationTable.getInstance();
    private boolean indexCommandScheduled;

    public ShootSequence(Shooter shooter, Indexer indexer, Drivetrain drivetrain, Limelight limelight, Command indexCommand) {
        this.indexCommand = indexCommand;
        this.shooter = shooter;
        this.indexer = indexer;
        this.limelight = limelight;
        addRequirements(shooter, indexer, drivetrain);
        addCommands(
                sequence(
                        new InstantCommand(() -> {
                            indexCommandScheduled = indexCommand.isScheduled();
                            indexCommand.cancel();
                        }),
                        parallel(
                                sequence(
                                        new InstantCommand(() -> limelight.setLeds(true)),
                                        new WaitCommand(0.2),
                                        new InstantCommand(() -> {
                                            double distanceFromTarget = limelight.getCalculatedDistance();
                                            double frontVelocity = interpolationTable.frontFlywheelInterpolator.getInterpolatedValue(distanceFromTarget);
                                            double rearVelocity = interpolationTable.rearFlywheelInterpolator.getInterpolatedValue(distanceFromTarget);
                                            System.out.println("Interpolated output for distance " + distanceFromTarget + " front " + frontVelocity + " rear " + rearVelocity);
                                            shooter.setVelocityRPM(shooter.shooterMotorFront, frontVelocity);
                                            shooter.setVelocityRPM(shooter.shooterMotorRear, rearVelocity);
                                        }, shooter)
                                ),
                                new ConditionalCommand( // Shoot if target isn't found, otherwise lineup and shoot
                                        new PointToTarget(drivetrain, limelight),
                                        new InstantCommand(),
                                        limelight::isTargetFound
                                )
                        ),
                        new WaitCommand(1),
                        new WaitUntilCommand(shooter::isSpunUp),
                        new InstantCommand(() -> indexer.set(Indexer.IndexerState.FORWARD_FULL, Indexer.IndexerState.FORWARD_FULL)),
                        new WaitCommand(2), // TODO: Maybe fix this?
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
        System.out.println("Stopping ShootSequence");
        shooter.stop();
        indexer.stop();
        limelight.setLeds(false);

        if (indexCommandScheduled) {
            indexCommand.schedule();
        }
    }
}
