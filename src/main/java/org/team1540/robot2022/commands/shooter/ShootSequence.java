package org.team1540.robot2022.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import java.net.Socket;
import org.team1540.robot2022.InterpolationTable;
import org.team1540.robot2022.commands.drivetrain.Drivetrain;
import org.team1540.robot2022.commands.drivetrain.PointToTarget;
import org.team1540.robot2022.commands.hood.Hood;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.commands.indexer.Indexer.IndexerState;
import org.team1540.robot2022.commands.intake.Intake;
import org.team1540.robot2022.utils.FeatherClient;
import org.team1540.robot2022.utils.LIDAR;
import org.team1540.robot2022.utils.Limelight;
import org.team1540.robot2022.utils.RepeatCommand;

public class ShootSequence extends SequentialCommandGroup {
    private final Shooter shooter;
    private final Indexer indexer;
    private final Limelight limelight;
    private final InterpolationTable interpolationTable = InterpolationTable.getInstance();

    private double limelightDistance;
    private double lidarDistance;
    private double frontVelocity;
    private double rearVelocity;
    private boolean hoodState; // New state to set the hood to

    private boolean hasInitializedFlywheels = false;

    public ShootSequence(Shooter shooter, Indexer indexer, Drivetrain drivetrain, Hood hood, Intake intake, Limelight limelight, LIDAR lidar) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.limelight = limelight;

        addRequirements(shooter, indexer, drivetrain);
        addCommands(
                sequence(
                        deadline(
                                new ConditionalCommand( // Shoot if target isn't found, otherwise lineup and shoot
                                        new PointToTarget(drivetrain, limelight).withTimeout(2),
                                        new WaitUntilCommand(() -> hasInitializedFlywheels), // Make sure flywheel setup runs at least once
                                        limelight::isTargetFound
                                ),
                                sequence(
                                        new InstantCommand(() -> limelight.setLeds(true)),
                                        new WaitCommand(0.2),
                                        new RepeatCommand(() -> {
                                            limelightDistance = limelight.getCalculatedDistance();
                                            lidarDistance = lidar.getDistance();

                                            if (lidarDistance < SmartDashboard.getNumber("shooter/minFarDistance", 93)) {
                                                hoodState = true;
                                                intake.setFold(true);
                                                frontVelocity = interpolationTable.frontFlywheelInterpolator.getInterpolatedValue(limelightDistance);
                                                rearVelocity = interpolationTable.rearFlywheelInterpolator.getInterpolatedValue(limelightDistance);
                                            } else { // Tarmac shot
                                                hoodState = false;
                                                frontVelocity = InterpolationTable.hubFront;
                                                rearVelocity = InterpolationTable.hubRear;
                                            }
                                            hood.set(hoodState);

                                            // Used for tuning:
                                            // frontVelocity = SmartDashboard.getNumber("shooter/tuning/frontRPM", 0);
                                            // rearVelocity = SmartDashboard.getNumber("shooter/tuning/rearRPM", 0);
                                            shooter.setVelocityRPM(shooter.shooterMotorFront, frontVelocity);
                                            shooter.setVelocityRPM(shooter.shooterMotorRear, rearVelocity);
                                            hasInitializedFlywheels = true;
                                        }, shooter)
                                )
                        ),
                        new WaitCommand(1),
                        new WaitUntilCommand(shooter::isSpunUp),
                        indexer.commandStop(),
                        indexer.commandSet(Indexer.IndexerState.FORWARD_FULL, Indexer.IndexerState.OFF), // Run top indexer
                        new WaitUntilCommand(() -> !indexer.getTopSensor()),
                        new WaitCommand(SmartDashboard.getNumber("shooter/tuning/waitAfterFirstBall", 1)), // Wait for top ball to leave and shooter to recover

                        new WaitUntilCommand(shooter::isSpunUp),
                        indexer.commandSet(Indexer.IndexerState.FORWARD_FULL, Indexer.IndexerState.FORWARD_FULL), // Run bottom indexer to shoot bottom ball
                        new WaitUntilCommand(() -> !indexer.getTopSensor()),
                        new WaitCommand(2), // Wait for bottom ball to leave TODO: Can we decrease this?
                        indexer.commandSet(Indexer.IndexerState.OFF, Indexer.IndexerState.OFF),
                        shooter.commandStop(),
                        new InstantCommand(() -> FeatherClient.recordShot(limelightDistance, lidarDistance, frontVelocity, rearVelocity, hoodState))
                )
        );
    }

    @Override
    public void end(boolean isInterrupted) {
        System.out.println("Stopping ShootSequence");
        shooter.stop();
        indexer.stop();
        limelight.setLeds(false);
    }
}
