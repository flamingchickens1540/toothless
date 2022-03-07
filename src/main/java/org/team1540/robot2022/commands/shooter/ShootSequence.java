package org.team1540.robot2022.commands.shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2022.InterpolationTable;
import org.team1540.robot2022.commands.drivetrain.Drivetrain;
import org.team1540.robot2022.commands.drivetrain.PointToTarget;
import org.team1540.robot2022.commands.hood.Hood;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.commands.intake.Intake;
import org.team1540.robot2022.commands.shooter.Shooter.ShooterProfile;
import org.team1540.robot2022.utils.ChickenShuffleboard;
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
                    if (ChickenShuffleboard.ShooterTab.Tuning.enableManualSetpoints.getBoolean(false) && !DriverStation.isFMSAttached()) { // Make sure this is never going during a competition
                        hoodState = ChickenShuffleboard.ShooterTab.Tuning.manualSetpointHood.getBoolean(true);
                        frontVelocity = ChickenShuffleboard.ShooterTab.Tuning.manualSetpointFront.getDouble(1000);
                        rearVelocity = ChickenShuffleboard.ShooterTab.Tuning.manualSetpointRear.getDouble(1000);

                    } else if (this.profile == Shooter.ShooterProfile.FAR) {
                        hoodState = true;
                        frontVelocity = interpolationTable.frontFlywheelInterpolator.getInterpolatedValue(limelightDistance);
                        rearVelocity = interpolationTable.rearFlywheelInterpolator.getInterpolatedValue(limelightDistance);
                    } else if (this.profile == Shooter.ShooterProfile.HUB) {
                        hoodState = false;
                        frontVelocity = ChickenShuffleboard.ShooterTab.Presets.hubFront.getDouble(InterpolationTable.hubFront);
                        rearVelocity  = ChickenShuffleboard.ShooterTab.Presets.hubRear.getDouble(InterpolationTable.hubRear);
                    } else if (this.profile == Shooter.ShooterProfile.TARMAC) {
                        hoodState = false;
                        frontVelocity = ChickenShuffleboard.ShooterTab.Presets.tarmacFront.getDouble(InterpolationTable.tarmacFront);
                        rearVelocity = ChickenShuffleboard.ShooterTab.Presets.tarmacRear.getDouble(InterpolationTable.tarmacRear);
                    } else if (this.profile == Shooter.ShooterProfile.LOWGOAL) {
                        hoodState = false;
                        frontVelocity = ChickenShuffleboard.ShooterTab.Presets.lowGoalFront.getDouble(InterpolationTable.lowGoalFront);
                        rearVelocity = ChickenShuffleboard.ShooterTab.Presets.lowGoalRear.getDouble(InterpolationTable.lowGoalRear);
                    }


                    hood.set(hoodState);
                    shooter.setVelocityRPM(shooter.shooterMotorFront, frontVelocity);
                    shooter.setVelocityRPM(shooter.shooterMotorRear, rearVelocity);
                }),
                FeatherClient.commandRecordShot(limelightDistance, lidarDistance, frontVelocity, rearVelocity, hoodState, this.profile),

                new ConditionalCommand( // Shoot if target isn't found, otherwise lineup and shoot
                        new PointToTarget(drivetrain, limelight).withTimeout(2),
                        new InstantCommand(),
                        () -> limelight.isTargetFound() && !this.profile.equals(ShooterProfile.HUB) && pointToTarget
                ),
                new WaitCommand(0.25),
                new WaitUntilCommand(shooter::isSpunUp),
                new ShooterFeedSequence(indexer, shooter)
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
        if (limelight != null) {
            limelight.setLeds(false);
        }
    }
}
