package org.team1540.robot2022.utils;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import org.team1540.robot2022.RamseteConfig;
import org.team1540.robot2022.commands.drivetrain.Drivetrain;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.commands.intake.Intake;
import org.team1540.robot2022.commands.intake.IntakeSequence;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import org.team1540.robot2022.commands.shooter.Shooter;

public class AutoHelper {



    /**
     * Returns a ParallelDeadlineGroup that runs intake and indexer while following the given trajectory
     * @param drivetrain The drivetrain subsystem
     * @param intake The intake subsystem
     * @param indexer The indexer subsystem
     * @param trajectoryName the path of the trajectory file relative to `deploy/paths`
     * @return The SequentialCommandGroup
     */
    public static ParallelDeadlineGroup runPath(Drivetrain drivetrain, Intake intake, Indexer indexer, Shooter shooter, String trajectoryName) {
        System.out.println(trajectoryName);
        return new ParallelDeadlineGroup(
                getRamseteCommand(drivetrain, trajectoryName),             // Path follow to collect first bal
                new IntakeSequence(intake, indexer, shooter)
        );
    }

        /**
     * Returns a new {@link RamseteCommand} with constants pre-filled for
     * convenience
     * 
     * @param drivetrain The drivetrain subsystem
     * @param trajectory The trajectory to follow with the command
     * @return A RamseteCommand to follow the trajectory
     */
    public static RamseteCommand getRamseteCommand(Drivetrain drivetrain, Trajectory trajectory) {
        return getRamseteCommand(drivetrain, trajectory, RamseteConfig.leftPID, RamseteConfig.rightPID);
    }

    /**
     * Returns a new {@link RamseteCommand} with constants pre-filled for
     * convenience
     * 
     * @param drivetrain The drivetrain subsystem
     * @param trajectory The trajectory to follow with the command
     * @param leftPID    A PID controller to use for the left wheels
     * @param rightPID   A PID controller to use for the right wheels
     * @return A RamseteCommand to follow the trajectory
     */
    public static RamseteCommand getRamseteCommand(Drivetrain drivetrain, Trajectory trajectory, PIDController leftPID,
            PIDController rightPID) {
        return new RamseteCommand(
                trajectory,
                drivetrain::getPose,
                RamseteConfig.ramseteController,
                RamseteConfig.feedForward,
                RamseteConfig.kDriveKinematics,
                drivetrain::getWheelSpeeds,
                leftPID,
                rightPID,
                drivetrain::setVolts,
                drivetrain);
    }

    /**
     * Returns a new {@link RamseteCommand} with trajectory and constants pre-filled
     * for convenience
     * 
     * @param drivetrain     The drivetrain subsystem
     * @param trajectoryName The name of the trajectory file relative to the
     *                       <code>deploy/paths</code> directory.
     * @return A RamseteCommand to follow the trajectory
     */
    public static RamseteCommand getRamseteCommand(Drivetrain drivetrain, String trajectoryName) {
        Trajectory trajectory = getTrajectory(trajectoryName);
        drivetrain.dashboardField.getObject("trajectory/"+trajectoryName).setTrajectory(trajectory);
        return getRamseteCommand(drivetrain, trajectory);
    }

    public static Trajectory getTrajectory(String trajectoryName) {
        Path trajectoryPath = getTrajectoryPath(trajectoryName);
        Trajectory trajectory;

        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {

            File file = trajectoryPath.toFile();

            if (!file.exists()) {
                DriverStation.reportError("File does not exist! "+ex.getLocalizedMessage(), ex.getStackTrace());
            } else if (!file.canRead()) {
                DriverStation.reportError("File can't be read! "+ex.getLocalizedMessage(), ex.getStackTrace());
            }
            trajectory = new Trajectory();
        }
        if (trajectory.getStates().size() < 2)
            throw new NullPointerException("Trajectory has too few points!");
        return trajectory;
    }

    public static Path getTrajectoryPath(String trajectoryName) {
        return Filesystem.getDeployDirectory().toPath().resolve("paths").resolve(trajectoryName);
    }
}
