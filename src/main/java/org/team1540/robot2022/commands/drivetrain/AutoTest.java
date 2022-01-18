package org.team1540.robot2022.commands.drivetrain;

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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class AutoTest extends CommandBase {
    private String trajectoryJSON = "paths/AutoTest.wpilib.json";
    private Trajectory trajectory;

    private RamseteCommand command;

    private DriveTrain driveTrain;

    public AutoTest(DriveTrain driveTrain) {
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }

        this.driveTrain = driveTrain;
        command = getRamseteCommand(trajectory);
        addRequirements(driveTrain);
    }

    private RamseteCommand getRamseteCommand(Trajectory trajectory) {
        return new RamseteCommand(
                trajectory,
                driveTrain::getPose,
                new RamseteController(RamseteConfig.kRamseteB, RamseteConfig.kRamseteZeta),
                new SimpleMotorFeedforward(
                        RamseteConfig.ksVolts,
                        RamseteConfig.kvVoltSecondsPerMeter,
                        RamseteConfig.kaVoltSecondsSquaredPerMeter),
                RamseteConfig.kDriveKinematics,
                driveTrain::getWheelSpeeds,
                new PIDController(RamseteConfig.kPDriveVel, 0, 0),
                new PIDController(RamseteConfig.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                driveTrain::tankDriveVolts,
                driveTrain);
    }

    @Override
    public void initialize() {
        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }
}
