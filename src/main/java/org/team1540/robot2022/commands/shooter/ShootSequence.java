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
import org.team1540.robot2022.utils.LIDAR;
import org.team1540.robot2022.utils.Limelight;

public class ShootSequence extends SequentialCommandGroup {
    private final Shooter shooter;
    private final Indexer indexer;
    private final Limelight limelight;
    private final InterpolationTable interpolationTable = InterpolationTable.getInstance();

    public Shooter.ShooterProfile profile;

    private double limelightDistance;
    private double lidarDistance;
    private double frontVelocity;
    private double rearVelocity;
    private boolean hoodState; // New state to set the hood to

    public ShootSequence(Shooter shooter, Indexer indexer, Drivetrain drivetrain, Hood hood, Intake intake, Limelight limelight, LIDAR lidar, Shooter.ShooterProfile profile, boolean shootWithoutTarget) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.limelight = limelight;
        this.profile = profile;
        // this.profile = Shooter.ShooterProfile.FAR;

        addRequirements(shooter, indexer, drivetrain);
        addCommands(
                new PrintCommand("Starting shoot sequence with profile " + this.profile.toString()),
                new InstantCommand(() -> limelight.setLeds(true)),
                new WaitCommand(0.2), // Wait for limelight to acquire target after LEDs turn on
                new InstantCommand(() -> {
//                    limelightDistance = limelight.getCalculatedDistance();
//                    lidarDistance = lidar.getDistance();
//
//                    // Positional argument shooter, not the field
//                    if (profile == Shooter.ShooterProfile.AUTOMATIC) {
//                        if (lidarDistance < SmartDashboard.getNumber("shooter/profiles/maxHub", 10)) {
//                            this.profile = Shooter.ShooterProfile.HUB;
//                        } else if (lidarDistance < SmartDashboard.getNumber("shooter/profiles/maxTarmac", 93)) {
//                            this.profile = Shooter.ShooterProfile.TARMAC;
//                        } else {
//                            this.profile = Shooter.ShooterProfile.FAR;
//                        }
//                    }

                    if (this.profile == Shooter.ShooterProfile.FAR) {
                        hoodState = true;
                        frontVelocity = interpolationTable.frontFlywheelInterpolator.getInterpolatedValue(limelightDistance);
                        rearVelocity = interpolationTable.rearFlywheelInterpolator.getInterpolatedValue(limelightDistance);
                    } else if (this.profile == Shooter.ShooterProfile.HUB) {
                        hoodState = false;
                        frontVelocity = InterpolationTable.hubFront;
                        rearVelocity = InterpolationTable.hubRear;
                    } else if (this.profile == Shooter.ShooterProfile.TARMAC) {
                        hoodState = false;
                        frontVelocity = InterpolationTable.tarmacFront;
                        rearVelocity = InterpolationTable.tarmacRear;
                    } else if (this.profile == Shooter.ShooterProfile.LOWGOAL) {
                        hoodState = false;
                        frontVelocity = InterpolationTable.lowGoalFront;
                        rearVelocity = InterpolationTable.lowGoalRear;
                    }

                    // Used for tuning:
                    // hoodState = true;
                    // frontVelocity = SmartDashboard.getNumber("shooter/tuning/frontRPM", 0);
                    // rearVelocity = SmartDashboard.getNumber("shooter/tuning/rearRPM", 0);

                    hood.set(hoodState);
                    shooter.setVelocityRPM(shooter.shooterMotorFront, frontVelocity);
                    shooter.setVelocityRPM(shooter.shooterMotorRear, rearVelocity);
                }),
                new InstantCommand(() -> FeatherClient.recordShot(limelightDistance, lidarDistance, frontVelocity, rearVelocity, hoodState, profile)),

                new ConditionalCommand( // Shoot if target isn't found, otherwise lineup and shoot
                        new PointToTarget(drivetrain, limelight).withTimeout(2),
                        new InstantCommand(() -> {
//                            if (!shootWithoutTarget) {
//                                this.end(true);
//                            }
                        }),
                        limelight::isTargetFound
                ),
                new WaitCommand(0.25),
                new WaitUntilCommand(shooter::isSpunUp),
                indexer.commandStop(),
                indexer.commandSet(Indexer.IndexerState.FORWARD_FULL, Indexer.IndexerState.OFF), // Run top indexer
                new WaitUntilCommand(() -> !indexer.getTopSensor()),
                new WaitCommand(SmartDashboard.getNumber("shooter/tuning/waitAfterFirstBall", 1)), // Wait for top ball to leave and shooter to recover
                indexer.commandSet(Indexer.IndexerState.OFF, Indexer.IndexerState.OFF),

                // Shoot second ball
                new WaitCommand(0.5),
                indexer.commandSet(Indexer.IndexerState.FORWARD, Indexer.IndexerState.FORWARD), // Move bottom ball up
                new WaitUntilCommand(indexer::getTopSensor), // Stop when ball is up high TODO: Standby here?
                indexer.commandSet(Indexer.IndexerState.OFF, Indexer.IndexerState.OFF),

                new WaitUntilCommand(shooter::isSpunUp),
                indexer.commandSet(Indexer.IndexerState.FORWARD_FULL, Indexer.IndexerState.FORWARD_FULL), // Run bottom indexer to shoot bottom ball
                new WaitUntilCommand(() -> !indexer.getTopSensor()).andThen(new WaitCommand(0.5)), // Wait for bottom ball to leave
                indexer.commandSet(Indexer.IndexerState.OFF, Indexer.IndexerState.OFF),
                shooter.commandStop()
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
