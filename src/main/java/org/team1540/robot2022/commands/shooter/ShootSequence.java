package org.team1540.robot2022.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2022.InterpolationTable;
import org.team1540.robot2022.commands.drivetrain.Drivetrain;
import org.team1540.robot2022.commands.drivetrain.PointToTarget;
import org.team1540.robot2022.commands.hood.Hood;
import org.team1540.robot2022.commands.indexer.IndexCommand;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.commands.intake.Intake;
import org.team1540.robot2022.utils.Limelight;
import org.team1540.robot2022.utils.RepeatCommand;

public class ShootSequence extends SequentialCommandGroup {
    private final Command indexCommand;
    private final Shooter shooter;
    private final Indexer indexer;
    private final Limelight limelight;
    private final InterpolationTable interpolationTable = InterpolationTable.getInstance();
    private boolean indexCommandScheduled;

    public ShootSequence(Shooter shooter, Indexer indexer, Drivetrain drivetrain, Hood hood, Intake intake, Limelight limelight, Command indexCommand) {
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
                                            double frontVelocity;
                                            double rearVelocity;
                                            if (distanceFromTarget > 98) { // Far shot, 98 inches TODO: needs tuning and LIDAR
                                                hood.set(true);
                                                intake.setFold(true);
                                                frontVelocity = interpolationTable.frontFlywheelInterpolator.getInterpolatedValue(distanceFromTarget);
                                                rearVelocity = interpolationTable.rearFlywheelInterpolator.getInterpolatedValue(distanceFromTarget);
                                            } else { // Tarmac shot
                                                hood.set(false);
                                                frontVelocity = InterpolationTable.tarmacFront;
                                                rearVelocity = InterpolationTable.tarmacRear;
                                            }

//                                            double frontVelocity = SmartDashboard.getNumber("shooter/tuning/frontRPM", 0);
//                                            double rearVelocity = SmartDashboard.getNumber("shooter/tuning/rearRPM", 0);
//                                            System.out.println("Setting output with distance " + distanceFromTarget + " front " + frontVelocity + " rear " + rearVelocity);
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
                        new InstantCommand(() -> indexer.set(Indexer.IndexerState.FORWARD_FULL, Indexer.IndexerState.OFF)), // Run top indexer
                        new WaitUntilCommand(() -> !indexer.getTopSensor()),
                        new WaitCommand(SmartDashboard.getNumber("shooter/tuning/waitAfterFirstBall", 1)), // Wait for top ball to leave and shooter to recover

                        new WaitUntilCommand(shooter::isSpunUp),
                        new InstantCommand(() -> indexer.set(Indexer.IndexerState.FORWARD_FULL, Indexer.IndexerState.FORWARD_FULL)), // Run bottom indexer to shoot bottom ball
                        new WaitUntilCommand(() -> !indexer.getTopSensor()),
                        new WaitCommand(2), // Wait for bottom ball to leave
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
