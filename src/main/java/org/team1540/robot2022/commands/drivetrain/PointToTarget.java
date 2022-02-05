package org.team1540.robot2022.commands.drivetrain;

import org.team1540.robot2022.utils.Limelight;
import org.team1540.robot2022.utils.MiniPID;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PointToTarget extends CommandBase {
    private final double LIMELIGHT_HORIZONTAL_FOV = 29.8;
    // private final double proportion = 0.2;
    // private final double maxTurnSpeed = 0.1;
    private final DriveTrain drivetrain;
    private final Limelight limelight;

    // A little testing says kP=0.7 and kD=0.4 are fairly strong.
    private final MiniPID pid = new MiniPID(1, 0, 0);

    public PointToTarget(DriveTrain drivetrain, Limelight limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        double p = SmartDashboard.getNumber("pointToTarget/kP", 0);
        double i = SmartDashboard.getNumber("pointToTarget/kI", 0);
        double d = SmartDashboard.getNumber("pointToTarget/kD", 0);
        pid.setPID(p, i, d);
        pid.setSetpoint(0);
    }

    private double getHorizontalDistanceToTarget() {
        return limelight.getTargetAngles().x;
    }

    private double getError(double distanceToTarget) {
        return Math.abs(distanceToTarget) / LIMELIGHT_HORIZONTAL_FOV;
    }

    public void execute() {
        Vector2d lmAngles = limelight.getTargetAngles();
        if (Math.abs(lmAngles.x) > 2) {
            
            double distanceToTarget = getHorizontalDistanceToTarget();
            double pidOutput = pid.getOutput(getError(distanceToTarget));
            double multiplier = lmAngles.x > 0 ? 1 : -1;
            
            // System.out.println("wants to turn at " + pidOutput + "% with tx: " + distanceToTarget);
            SmartDashboard.putNumber("pointToTarget/pidOutput", pidOutput);
            SmartDashboard.putNumber("pointToTarget/distanceToTarget", distanceToTarget);

            if (pidOutput > .8) {
                pidOutput = 0;
                System.out.println("Too high a PID value");
                this.end(false);
            }

            double valueL = multiplier * -pidOutput;
            double valueR = multiplier * pidOutput;
            System.out.println(valueL + ", " + valueR);
            drivetrain.setPercent(valueL, valueR);
        }
    }

    public void end(boolean isInterrupted) {
        drivetrain.setPercent(0, 0);
    }
}
