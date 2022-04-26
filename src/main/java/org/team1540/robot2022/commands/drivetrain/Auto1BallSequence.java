package org.team1540.robot2022.commands.drivetrain;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.team1540.robot2022.commands.hood.Hood;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.commands.intake.Intake;
import org.team1540.robot2022.commands.shooter.ShootSequence;
import org.team1540.robot2022.commands.shooter.Shooter;
import org.team1540.robot2022.commands.vision.Vision;
import org.team1540.robot2022.utils.*;
import org.team1540.robot2022.utils.AutoHelper.AutoPath;


public class Auto1BallSequence extends AutoSequence {

    public Auto1BallSequence(Drivetrain drivetrain, Intake intake, Indexer indexer, Vision vision, Shooter shooter, Hood hood, Limelight limelight, LIDAR lidar, NavX navx, boolean shouldTaxi) {
        Trajectory trajectory = AutoPath.auto1Ball.trajectory;
        if (shouldTaxi) {
            addPaths(AutoPath.auto1Ball);
        }

        addCommands(
                new InstantCommand(() -> drivetrain.resetOdometry(AutoPath.auto1Ball.trajectory.getInitialPose())), // Reset to path's starting pose (for any starting location)            

                shooter.commandSetVelocity(AutoPath.auto1Ball.frontSetpoint, AutoPath.auto1Ball.rearSetpoint),
                hood.commandSet(AutoPath.auto1Ball.hoodState),
                new InstantCommand(() -> indexer.setStandby(false)),
                indexer.commandSet(Indexer.IndexerState.FORWARD, Indexer.IndexerState.FORWARD),
                parallel(
                        new ConditionalCommand(
                                AutoHelper.getRamseteCommand(drivetrain, trajectory),  // Drive backwards
                                new InstantCommand(),
                                () -> shouldTaxi
                        ),
                        sequence(
                                new WaitUntilCommand(indexer::getTopSensor),
                                indexer.commandSet(Indexer.IndexerState.OFF, Indexer.IndexerState.OFF)
                        )

                ),
                new ShootSequence(shooter,
                        indexer,
                        drivetrain,
                        hood,
                        intake,
                        vision,
                        limelight,
                        lidar,
                        navx,
                        Shooter.ShooterProfile.FAR,
                        true, false, null)
        );
    }
}
