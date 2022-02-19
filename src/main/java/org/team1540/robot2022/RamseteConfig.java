package org.team1540.robot2022;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;

import org.team1540.robot2022.commands.drivetrain.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class RamseteConfig {
    // Calculated in frc-characterization
    public static final double ksVolts = 0.127;
    public static final double kvVoltSecondsPerMeter = 2.6;
    public static final double kaVoltSecondsSquaredPerMeter = 0.292;

    // Ramsete PID controllers
    public static final double kPDriveVel = 0.7;

    private static final double kTrackwidthMeters = 0.67978793613;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            kTrackwidthMeters);

    // private static final double kWheelCircumference = 0.4863499587;
    // private static final double kEncoderPPR = 512;
    // private static final double encoderMetersPerTick = kWheelCircumference /
    // kEncoderPPR;

    // Motion control
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double kMaxSpeedMetersPerSecond = 0.2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.1;

    public static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter),
            kDriveKinematics,
            10);

    // RamseteCommand Defaults
    public static final RamseteController ramseteController = new RamseteController(kRamseteB, kRamseteZeta);
    public static final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter,
            kaVoltSecondsSquaredPerMeter);
    public static final PIDController leftPID = new PIDController(SmartDashboard.getNumber("ramsetePID/kP", 0.5), 0, 0);
    public static final PIDController rightPID = new PIDController(SmartDashboard.getNumber("ramsetePID/kP", 0.5), 0,
            0);

    /**
     * Returns a new {@link RamseteCommand} with constants pre-filled for
     * convenience
     * 
     * @param drivetrain The drivetrain subsystem
     * @param trajectory The trajectory to follow with the command
     * @return A RamseteCommand to follow the trajectory
     */
    public static RamseteCommand getRamseteCommand(Drivetrain drivetrain, Trajectory trajectory) {
        return getRamseteCommand(drivetrain, trajectory, leftPID, rightPID);
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
        return getRamseteCommand(drivetrain, trajectory);
    }

    public static Path getTrajectoryPath(String trajectoryName) {
        return Filesystem.getDeployDirectory().toPath().resolve("paths").resolve(trajectoryName);
    }
}