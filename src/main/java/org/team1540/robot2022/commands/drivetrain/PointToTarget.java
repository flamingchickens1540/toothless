package org.team1540.robot2022.commands.drivetrain;

import org.team1540.robot2022.utils.Limelight;
import org.team1540.robot2022.utils.MiniPID;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PointToTarget extends CommandBase {
    private final double LIMELIGHT_HORIZONTAL_FOV = 29.8;
    private final Drivetrain drivetrain;
    private final Limelight limelight;
    private final MedianFilter medianFilter = new MedianFilter(10);

    // A little testing says kP=0.7 and kD=0.4 are fairly strong.
    private final MiniPID pid = new MiniPID(1, 0, 0);

    public PointToTarget(Drivetrain drivetrain, Limelight limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        double p = SmartDashboard.getNumber("pointToTarget/kP", 0.7);
        double i = SmartDashboard.getNumber("pointToTarget/kI", 0);
        double d = SmartDashboard.getNumber("pointToTarget/kD", 0.4);
        limelight.setLeds(true);
        pid.setPID(p, i, d);
        pid.setSetpoint(0);
    }

    private double getError(double distanceToTarget) {
        return Math.abs(distanceToTarget) / LIMELIGHT_HORIZONTAL_FOV;
    }

    public void execute() {
        // Grab angles
        Vector2d lmAngles = limelight.getTargetAngles();
        // Calculate median
        double medX = medianFilter.calculate(lmAngles.x);
        // Run PID calculations to turn
        if (Math.abs(medX) > SmartDashboard.getNumber("pointToTarget/targetDeadzoneDegrees", 2)) {

            double pidOutput = pid.getOutput(getError(medX));
            double multiplier = lmAngles.x > 0 ? 1 : -1;

            SmartDashboard.putNumber("pointToTarget/pidOutput", pidOutput);
            SmartDashboard.putNumber("pointToTarget/degreeDistanceToTargetRaw", lmAngles.x);
            SmartDashboard.putNumber("pointToTarget/degreeDistanceToTargetMedian", medX);

            if (pidOutput > SmartDashboard.getNumber("pointToTarget/pidClamp", 0.8)) {
                pidOutput = 0;
                SmartDashboard.putBoolean("pointToTarget/isClamping", true);
                this.end(false);
            } else {
                SmartDashboard.putBoolean("pointToTarget/isClamping", false);
            }

            double valueL = multiplier * -pidOutput;
            double valueR = multiplier * pidOutput;

            drivetrain.setPercent(valueL, valueR);
        }
    }

    public void end(boolean isInterrupted) {
        drivetrain.stopMotors();
        limelight.setLeds(false);
    }
}
