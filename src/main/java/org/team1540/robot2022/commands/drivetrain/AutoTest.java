package org.team1540.robot2022.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2022.Constants;
import org.team1540.robot2022.RamseteConfig;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;

public class AutoTest extends SequentialCommandGroup {
    private final String trajectoryJSON = "paths/test.path2.wpilib.json";
    private final Drivetrain drivetrain;
    private Trajectory trajectory = new Trajectory();

    public AutoTest(Drivetrain drivetrain) {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            File trajectoryFile = trajectoryPath.toFile();
            DriverStation.reportError("File Exists: " + trajectoryFile.exists(), false);
            DriverStation.reportError("File Readable: " + trajectoryFile.canRead(), false);
            DriverStation.reportError("File Writable: " + trajectoryFile.canWrite(), false);
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }

        this.drivetrain = drivetrain;
        addCommands(getRamseteCommand(trajectory), drivetrain.commandStop());
        addRequirements(drivetrain);
    }

    private RamseteCommand getRamseteCommand(Trajectory trajectory) {
        return new RamseteCommand(
                trajectory,                                                                 // The trajectory to follow.
                drivetrain::getPose,                                                        // A function that supplies the robot pose
                new RamseteController(RamseteConfig.kRamseteB, RamseteConfig.kRamseteZeta), // The RAMSETE controller used to follow the trajectory.
                new SimpleMotorFeedforward(                                                 // The feedforward to use for the drive.
                        Constants.DriveConstants.KS_VOLTS,
                        Constants.DriveConstants.KV_VOLT_SECONDS_PER_METER,
                        Constants.DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER),
                RamseteConfig.kDriveKinematics,                                             // The kinematics for the robot drivetrain.
                drivetrain::getWheelSpeeds,                                                 // A function that supplies the speeds of the left and right sides of the robot
                new PIDController(SmartDashboard.getNumber("ramsetePID/kP", 0.5), 0, 0),                          // Left PID controller
                new PIDController(SmartDashboard.getNumber("ramsetePID/kP", 0.5), 0, 0),                          // Right PID controller
                drivetrain::setVolts,                                                 // RamseteCommand passes volts to the callback
                drivetrain                                                                  // Subsystem requirements
        );
    }

    @Override
    public void initialize() {
        drivetrain.zeroHeading();
        drivetrain.resetOdometry(trajectory.getInitialPose());
        super.initialize();
    }
}
