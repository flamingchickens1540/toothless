package org.team1540.robot2022.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2022.InterpolationTable;
import org.team1540.robot2022.commands.drivetrain.Drivetrain;
import org.team1540.robot2022.commands.drivetrain.PointToTarget;
import org.team1540.robot2022.commands.hood.Hood;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.commands.intake.Intake;
import org.team1540.robot2022.commands.shooter.Shooter.ShooterProfile;
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

    public ShootSequence(Shooter shooter, Indexer indexer, Drivetrain drivetrain, Hood hood, Intake intake, Limelight limelight, LIDAR lidar, Shooter.ShooterProfile m_profile, boolean pointToTarget) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.limelight = limelight;
        this.profile = m_profile;
        // this.profile = Shooter.ShooterProfile.FAR;

        addRequirements(shooter, indexer, drivetrain);
        addCommands(
                new PrintCommand("Starting shoot sequence with profile " + this.profile),
                new ConditionalCommand(
                    sequence(
                        new InstantCommand(() -> limelight.setLeds(true)),
                        new WaitCommand(0.2)
                    ),
                    new InstantCommand(), 
                    () -> !this.profile.equals(ShooterProfile.HUB)
                ),
                new InstantCommand(() -> {
                    if (this.profile == Shooter.ShooterProfile.FAR) {
                        hoodState = true;
                        frontVelocity = interpolationTable.frontFlywheelInterpolator.getInterpolatedValue(limelightDistance);
                        rearVelocity = interpolationTable.rearFlywheelInterpolator.getInterpolatedValue(limelightDistance);
                    } else if (this.profile == Shooter.ShooterProfile.HUB) {
                        hoodState = false;
                        frontVelocity = SmartDashboard.getNumber("shooter/presets/hub/front", InterpolationTable.hubFront);
                        rearVelocity = SmartDashboard.getNumber("shooter/presets/hub/rear", InterpolationTable.hubRear);
                    } else if (this.profile == Shooter.ShooterProfile.TARMAC) {
                        hoodState = false;
                        frontVelocity = SmartDashboard.getNumber("shooter/presets/tarmac/front", InterpolationTable.tarmacFront);
                        rearVelocity = SmartDashboard.getNumber("shooter/presets/tarmac/rear", InterpolationTable.tarmacRear);
                    } else if (this.profile == Shooter.ShooterProfile.LOWGOAL) {
                        hoodState = false;
                        frontVelocity = SmartDashboard.getNumber("shooter/presets/lowgoal/front", InterpolationTable.lowGoalFront);
                        rearVelocity = SmartDashboard.getNumber("shooter/presets/lowgoal/rear", InterpolationTable.lowGoalRear);
                    }

                    // Used for tuning:
                    // hoodState = true;
                    // frontVelocity = SmartDashboard.getNumber("shooter/tuning/frontRPM", 0);
                    // rearVelocity = SmartDashboard.getNumber("shooter/tuning/rearRPM", 0);

                    hood.set(hoodState);
                    shooter.shooterMotorFront.setVelocityRPM(frontVelocity);
                    shooter.shooterMotorRear.setVelocityRPM(rearVelocity);
                }),
                new InstantCommand(() -> FeatherClient.recordShot(limelightDistance, lidarDistance, frontVelocity, rearVelocity, hoodState, this.profile)),

                new ConditionalCommand( // Shoot if target isn't found, otherwise lineup and shoot
                        new PointToTarget(drivetrain, limelight).withTimeout(2),
                        new InstantCommand(),
                        () -> limelight.isTargetFound() && !this.profile.equals(ShooterProfile.HUB) && pointToTarget
                ),
                new WaitCommand(0.25),
                new WaitUntilCommand(shooter::isSpunUp),
                indexer.commandStop(),
                indexer.commandSet(Indexer.IndexerState.FORWARD_FULL, Indexer.IndexerState.OFF), // Run top indexer
                new WaitUntilCommand(() -> !indexer.getTopSensor()),
                new WaitCommand(SmartDashboard.getNumber("shooter/tuning/waitAfterFirstBall", 0.5)), // Wait for top ball to leave and shooter to recover
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

    public void setProfile(ShooterProfile profile) {
        System.out.println("Setting profile "+profile);
        this.profile = profile;
    }

    @Override
    public void end(boolean isInterrupted) {
        System.out.println("Stopping ShootSequence");
        shooter.stop();
        indexer.stop();
        limelight.setLeds(false);
    }
}
