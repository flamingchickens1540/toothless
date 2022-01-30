package org.team1540.robot2022.commands.drivetrain;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;


import org.team1540.robot2022.RamseteConfig;

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

public class AutoTest extends SequentialCommandGroup {
    private String trajectoryJSON = "paths/test.path2.wpilib.json";
    private Trajectory trajectory = new Trajectory();

    private DriveTrain driveTrain;

    public AutoTest(DriveTrain driveTrain) {
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

        this.driveTrain = driveTrain;
        addCommands(getRamseteCommand(trajectory), driveTrain.commandStop());
        addRequirements(driveTrain);
    }

    private RamseteCommand getRamseteCommand(Trajectory trajectory) {
        return new RamseteCommand(
                trajectory,                                                                 // The trajectory to follow.
                driveTrain::getPose,                                                        // A function that supplies the robot pose
                new RamseteController(RamseteConfig.kRamseteB, RamseteConfig.kRamseteZeta), // The RAMSETE controller used to follow the trajectory.
                new SimpleMotorFeedforward(                                                 // The feedforward to use for the drive.
                        RamseteConfig.ksVolts,
                        RamseteConfig.kvVoltSecondsPerMeter,
                        RamseteConfig.kaVoltSecondsSquaredPerMeter),
                RamseteConfig.kDriveKinematics,                                             // The kinematics for the robot drivetrain.
                driveTrain::getWheelSpeeds,                                                 // A function that supplies the speeds of the left and right sides of the robot
                new PIDController(SmartDashboard.getNumber("drivePID/kP", 0.5), 0, 0),                          // Left PID controller
                new PIDController(SmartDashboard.getNumber("drivePID/kP", 0.5), 0, 0),                          // Right PID controller
                driveTrain::tankDriveVolts,                                                 // RamseteCommand passes volts to the callback
                driveTrain                                                                  // Subsystem requirements
        );
    }

    @Override
    public void initialize() {
        driveTrain.zeroHeading();
        driveTrain.resetOdometry(driveTrain.getPose());
        super.initialize();
    }
}
