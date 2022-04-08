package org.team1540.robot2022.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import org.team1540.robot2022.RamseteConfig;
import org.team1540.robot2022.Robot;
import org.team1540.robot2022.commands.drivetrain.Drivetrain;
import org.team1540.robot2022.commands.hood.Hood;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.commands.intake.Intake;
import org.team1540.robot2022.commands.intake.IntakeSequence;
import org.team1540.robot2022.commands.shooter.Shooter;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

public class AutoHelper {

    /**
     * Returns a ParallelDeadlineGroup that runs intake and indexer while following the given trajectory
     *
     * @param drivetrain The drivetrain subsystem
     * @param intake     The intake subsystem
     * @param indexer    The indexer subsystem
     * @return The SequentialCommandGroup
     */
    public static ParallelDeadlineGroup runPath(Drivetrain drivetrain, Intake intake, Indexer indexer, Shooter shooter, AutoPath path) {
        return new ParallelDeadlineGroup(
                getRamseteCommand(drivetrain, path),             // Path follow to collect first bal
                new IntakeSequence(intake, indexer, shooter)
        );
    }

    /**
     * Returns a ParallelDeadlineGroup that runs intake and indexer while following the given trajectory, and spins up the flywheels to a given speed
     *
     * @param drivetrain The drivetrain subsystem
     * @param intake     The intake subsystem
     * @param indexer    The indexer subsystem
     * @param path       the path to follow and get spinup constants from
     * @return The SequentialCommandGroup
     */
    public static ParallelDeadlineGroup runPathWithSpinup(Drivetrain drivetrain, Intake intake, Indexer indexer, Shooter shooter, Hood hood, AutoPath path) {
        return new ParallelDeadlineGroup(
                getRamseteCommand(drivetrain, path),             // Path follow to collect first bal
                new IntakeSequence(intake, indexer),

                // Prepare to shoot
                hood.commandSet(path.hoodState),                                      // Set hood
                // shooter.commandSetVelocity(path.frontSetpoint, path.rearSetpoint), // Spin up flywheels
                shooter.commandSetVelocity(2000, 2000)
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
     * @param path       the AutoPath to follow
     * @return A RamseteCommand to follow the trajectory
     */
    public static RamseteCommand getRamseteCommand(Drivetrain drivetrain, AutoPath path) {
        drivetrain.fieldWidget.addPath(path);
        return getRamseteCommand(drivetrain, path.trajectory);
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
        return getRamseteCommand(drivetrain, trajectory);
    }

    public static Trajectory getTrajectory(String trajectoryName) {
        Path trajectoryPath = getTrajectoryPath(trajectoryName);
        Trajectory trajectory;

        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            try {
                trajectory = TrajectoryUtil.fromPathweaverJson(Filesystem.getOperatingDirectory().toPath().resolve("PathWeaver/output").resolve(trajectoryName));
            } catch (IOException ex1) {
                if (Robot.isSimulation()) {
                    return new Trajectory(List.of(new Trajectory.State()));
                }
                File file = trajectoryPath.toFile();
                if (!file.exists()) {
                    DriverStation.reportError("File does not exist! " + ex.getLocalizedMessage(), ex.getStackTrace());
                } else if (!file.canRead()) {
                    DriverStation.reportError("File can't be read! " + ex.getLocalizedMessage(), ex.getStackTrace());
                }
                trajectory = new Trajectory();
            }


        }
        if (trajectory.getStates().size() < 2)
            throw new NullPointerException("Trajectory has too few points!");
        return trajectory;
    }

    public static Path getTrajectoryPath(String trajectoryName) {
        return Filesystem.getDeployDirectory().toPath().resolve("paths").resolve(trajectoryName);
    }

    public static class AutoPath {
        public static final AutoPath auto1Ball = new AutoPath("2ball.posA.path1.wpilib.json", 2200, 2500, true);
        public static final AutoPath auto2Ball1A = new AutoPath("2ball.posA.path1.wpilib.json", 2200, 2500, true);
        public static final AutoPath auto2Ball1B = new AutoPath("2ball.posB.path1.wpilib.json", 2200, 2500, true);
        public static final AutoPath auto2Ball2BM = new AutoPath("2ball.posBM.path2.wpilib.json", 2200, 2500, true);
        public static final AutoPath auto3Ball2 = new AutoPath("3ball.posA.path2.wpilib.json", 2500, 2800, true);
        public static final AutoPath auto5Ball2 = new AutoPath("5ball.posA.path2.wpilib.json", 2600, 2900, true);
        public static final AutoPath auto5Ball3 = new AutoPath("5ball.posA.path3.wpilib.json", 2400, 2800, true);
        public static final AutoPath auto5Ball4 = new AutoPath("5ball.posA.path4.wpilib.json", 2400, 2800, true);
        public final Trajectory trajectory;
        public final boolean hoodState;
        public final double frontSetpoint, rearSetpoint;
        public final String name;

        private AutoPath(String pathName, double frontSetpoint, double rearSetpoint, boolean hoodState) {
            this.trajectory = AutoHelper.getTrajectory(pathName);
            this.name = pathName;
            this.frontSetpoint = frontSetpoint;
            this.rearSetpoint = rearSetpoint;
            this.hoodState = hoodState;
        }
    }
}
