package org.team1540.robot2022.commands.drivetrain;

import org.team1540.robot2022.utils.Limelight;
import org.team1540.robot2022.utils.NavX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class OdometryResetSequence extends ParallelCommandGroup {
    //TODO: Set these to actual values
    public Pose2d zeroPose = new Pose2d(new Translation2d(1, 2), new Rotation2d(3, 4));
    public OdometryResetSequence(Drivetrain drivetrain, NavX navx, Limelight limelight) {
        boolean initialLightState = limelight.getLeds();
        addCommands(
            sequence( // Flash Limelight LEDs as confirmation
                new InstantCommand(() -> {limelight.setLeds(!initialLightState);}),
                new WaitCommand(0.5),
                new InstantCommand(() -> {limelight.setLeds(initialLightState);})
            ),
            new InstantCommand(() -> {drivetrain.resetOdometry(zeroPose);}) // Reset odometry to known starting pose

        );
    }



    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
