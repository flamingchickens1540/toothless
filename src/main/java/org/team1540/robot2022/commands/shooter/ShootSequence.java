package org.team1540.robot2022.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2022.InterpolationTable;
import org.team1540.robot2022.commands.drivetrain.Drivetrain;
import org.team1540.robot2022.commands.drivetrain.PointToTarget;
import org.team1540.robot2022.commands.hood.Hood;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.commands.intake.Intake;
import org.team1540.robot2022.utils.FeatherClient;
import org.team1540.robot2022.utils.Limelight;

public class ShootSequence extends SequentialCommandGroup {
    private final Shooter shooter;
    private final Indexer indexer;
    private final Limelight limelight;
    private final InterpolationTable interpolationTable = InterpolationTable.getInstance();

    public boolean shootFromHub = false;

    public ShootSequence(Shooter shooter, Indexer indexer, Drivetrain drivetrain, Hood hood, Intake intake, Limelight limelight) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.limelight = limelight;
        addRequirements(shooter, indexer, drivetrain);
        addCommands(
                sequence(
                        parallel(
                                sequence(
                                        new InstantCommand(() -> limelight.setLeds(true)),
                                        new WaitCommand(0.2),
                                        new InstantCommand(() -> {
                                            double distanceFromTarget = limelight.getCalculatedDistance();
                                            double frontVelocity;
                                            double rearVelocity;
                                            boolean hoodState; // New state to set the hood to

                                            if (!shootFromHub) {
                                                hoodState = true;
                                                intake.setFold(true);
                                                frontVelocity = interpolationTable.frontFlywheelInterpolator.getInterpolatedValue(distanceFromTarget);
                                                rearVelocity = interpolationTable.rearFlywheelInterpolator.getInterpolatedValue(distanceFromTarget);
                                            } else { // Tarmac shot
                                                hoodState = false;
                                                frontVelocity = InterpolationTable.hubFront;
                                                rearVelocity = InterpolationTable.hubRear;
                                            }
                                            hood.set(hoodState);

                                            // frontVelocity = SmartDashboard.getNumber("shooter/tuning/frontRPM", 0);
                                            // rearVelocity = SmartDashboard.getNumber("shooter/tuning/rearRPM", 0);
                                            // System.out.println("Setting output with distance " + distanceFromTarget + " front " + frontVelocity + " rear " + rearVelocity);
                                            SmartDashboard.putNumber("shooter/lastShot/frontRPM", frontVelocity);
                                            SmartDashboard.putNumber("shooter/lastShot/rearRPM", rearVelocity);
                                            SmartDashboard.putNumber("shooter/lastShot/distanceFromTarget", distanceFromTarget);
                                            SmartDashboard.putBoolean("shooter/lastShot/hoodState", hoodState);

                                            shooter.setVelocityRPM(shooter.shooterMotorFront, frontVelocity);
                                            shooter.setVelocityRPM(shooter.shooterMotorRear, rearVelocity);
                                            FeatherClient.recordShot(distanceFromTarget, frontVelocity, rearVelocity, hoodState);
                                        }, shooter)
                                ),
                                new ConditionalCommand( // Shoot if target isn't found, otherwise lineup and shoot
                                        new PointToTarget(drivetrain, limelight).withTimeout(2),
                                        new InstantCommand(),
                                        limelight::isTargetFound
                                )
                        ),

                        new WaitCommand(1),
                        new WaitUntilCommand(shooter::isSpunUp),
                        indexer.commandStop(),
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
    }
}
