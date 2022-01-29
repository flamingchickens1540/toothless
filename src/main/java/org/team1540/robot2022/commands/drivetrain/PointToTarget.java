package org.team1540.robot2022.commands.drivetrain;

import org.team1540.robot2022.utils.Limelight;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PointToTarget extends CommandBase {
    private final double turnSpeed = 0.1;
    private final DriveTrain drivetrain;
    private final Limelight limelight;

    public PointToTarget(DriveTrain drivetrain, Limelight limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        addRequirements(drivetrain);
    }

    public void execute() {
        Vector2d lmAngles = limelight.getTargetAngles();
        if (Math.abs(lmAngles.x) > 0.08) {
            System.out.println("wants to turn");

            double multiplier = lmAngles.x < 0 ? -1 : 1;
            
            double valueL = multiplier * turnSpeed;
            double valueR = multiplier * -turnSpeed;
            drivetrain.setPercent(valueL, valueR);
        }
    }

    public void end(boolean isInterrupted) {
        drivetrain.setPercent(0, 0);
    }
}
