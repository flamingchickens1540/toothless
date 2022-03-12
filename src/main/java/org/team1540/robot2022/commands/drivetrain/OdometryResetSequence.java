package org.team1540.robot2022.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team1540.robot2022.utils.AutoHelper;
import org.team1540.robot2022.utils.NavX;
import org.team1540.robot2022.utils.RevBlinkin;

public class OdometryResetSequence extends ParallelCommandGroup {
    public Pose2d zeroPose = AutoHelper.getTrajectory("reference.wpilib.json").getInitialPose();

    public OdometryResetSequence(Drivetrain drivetrain, NavX navx, RevBlinkin lights) {
        RevBlinkin.ColorPattern initialPattern = lights.getPattern();
        addCommands(
                sequence( // Flash Blinkin LEDs as confirmation
                        lights.commandSetPattern(RevBlinkin.ColorPattern.ORANGE),
                        new WaitCommand(0.5),
                        lights.commandSetPattern(initialPattern)
                ),
                sequence(
                        new InstantCommand(navx::zeroYaw),
                        new InstantCommand(() -> drivetrain.resetOdometry(zeroPose)) // Reset odometry to known starting pose
                ),
                new InstantCommand(() -> drivetrain.setNeutralMode(NeutralMode.Coast))
        );
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
