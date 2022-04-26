package org.team1540.robot2022.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team1540.robot2022.commands.vision.Vision;
import org.team1540.robot2022.utils.AutoHelper;
import org.team1540.robot2022.utils.Limelight;
import org.team1540.robot2022.utils.NavX;
import org.team1540.robot2022.utils.RevBlinkin;

public class OdometryResetSequence extends ParallelCommandGroup {
    public static Pose2d zeroPose = AutoHelper.getTrajectory("reference.wpilib.json").getInitialPose();

    public OdometryResetSequence(Drivetrain drivetrain, NavX navx, Vision vision, Limelight limelight, RevBlinkin lights) {
        RevBlinkin.ColorPattern initialPattern = lights.getPattern();
        addCommands(
                sequence(
                        new InstantCommand(() -> limelight.setLeds(Limelight.LEDMode.BLINK)),
                        new WaitCommand(0.5),
                        new InstantCommand(() -> limelight.setLeds(Limelight.LEDMode.OFF))
                ),
                getOdometryResetter(navx, drivetrain, vision),
                new InstantCommand(() -> drivetrain.setNeutralMode(NeutralMode.Coast))
        );
    }

    public static SequentialCommandGroup getOdometryResetter(NavX navx, Drivetrain drivetrain, Vision vision) {
        return new SequentialCommandGroup(
                new InstantCommand(navx::zeroYaw),
                new InstantCommand(() -> drivetrain.resetOdometry(zeroPose)),
                new InstantCommand(() -> drivetrain.setNeutralMode(NeutralMode.Coast)),
                new InstantCommand(vision::zeroPosition)
        );
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
