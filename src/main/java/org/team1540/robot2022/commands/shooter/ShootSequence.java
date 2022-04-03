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
import org.team1540.robot2022.commands.vision.Vision;
import org.team1540.robot2022.utils.*;

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

    public ShootSequence(Shooter shooter, Indexer indexer, Drivetrain drivetrain, Hood hood, Intake intake, Vision vision, Limelight limelight, LIDAR lidar, NavX navX, Shooter.ShooterProfile m_profile, boolean pointToTarget, boolean has2Balls, ChickenXboxController controller) {
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
                                new WaitCommand(0.2),
                                new ConditionalCommand( // Always lines up and shoots, given the new vision estimation
                                        new PointToTarget(drivetrain, vision, limelight, navX, controller),//.withTimeout(1),
                                        new InstantCommand(),
                                        () -> pointToTarget
                                ),
                                new InstantCommand(() -> {
                                    // TODO: TEST THIS!
                                    limelightDistance = vision.getCornerCalculatedDistance();
                                    lidarDistance = lidar.getDistance();
                                })
                        ),
                        new InstantCommand(),
                        () -> !this.profile.equals(ShooterProfile.HUB)
                ),
                new InstantCommand(() -> {
                    if (this.profile == ShooterProfile.TESTING) {
                        hoodState = true;
                        frontVelocity = SmartDashboard.getNumber("shooter/tuning/frontRPM", 0);
                        rearVelocity = SmartDashboard.getNumber("shooter/tuning/rearRPM", 0);
                    } else if (this.profile == Shooter.ShooterProfile.FAR) {
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

                    hood.set(hoodState);
                    shooter.setVelocityRPM(frontVelocity, rearVelocity);
                }),
                drivetrain.lights.commandSetPattern(RevBlinkin.ColorPattern.YELLOW),

                drivetrain.lights.commandSetPattern(RevBlinkin.ColorPattern.VIOLET),
                new InstantCommand(() -> FeatherClient.recordShot(this.limelightDistance, this.lidarDistance, this.frontVelocity, this.rearVelocity, this.hoodState, this.profile + "")),
                new ShooterFeedSequence(indexer, shooter, drivetrain.lights, has2Balls)
        );
    }


    public void setProfile(ShooterProfile profile) {
        System.out.println("Setting profile " + profile);
        this.profile = profile;
    }

    /**
     * Factory method for setting the profile
     *
     * @param profile The profile to set to
     * @return The InstantCommand that sets the profile
     */
    public InstantCommand commandSetProfile(ShooterProfile profile) {
        return new InstantCommand(() -> this.setProfile(profile));
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
