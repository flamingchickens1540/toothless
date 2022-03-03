package org.team1540.robot2022.commands.drivetrain;

import java.util.LinkedList;
import org.team1540.robot2022.utils.Limelight;
import org.team1540.robot2022.utils.MiniPID;
import org.team1540.robot2022.utils.NavX;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PointToTarget extends CommandBase {
    private final double LIMELIGHT_HORIZONTAL_FOV = 29.8;
    private final Drivetrain drivetrain;
    private final Limelight limelight;
    private final NavX navx;
    private LinkedList<Vector2d> pastPoses = new LinkedList<Vector2d>();

    // A little testing says kP=0.7 and kD=0.4 are fairly strong.
    private final MiniPID pid = new MiniPID(1, 0, 0);

    public PointToTarget(Drivetrain drivetrain, Limelight limelight, NavX navx) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.navx = navx;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        double p = SmartDashboard.getNumber("pointToTarget/kP", 0.05);
        double i = SmartDashboard.getNumber("pointToTarget/kI", 0);
        double d = SmartDashboard.getNumber("pointToTarget/kD", 0);
        limelight.setLeds(true);
        pid.setPID(p, i, d);
        pid.setSetpoint(0);
    }

    public void execute() {
        Vector2d lmAngles = limelight.getTargetAngles();
        pastPoses.add(lmAngles);
        if (pastPoses.size() > 10) {
            pastPoses.remove(0);
        }
        double avgX = 0;
        for (Vector2d pose : pastPoses) {
            avgX += pose.x;
        }
        avgX /= pastPoses.size();
        if (Math.abs(avgX) > SmartDashboard.getNumber("pointToTarget/targetDeadzoneDegrees", 2)) {

            double degreeSetpoint = navx.getAngle()+avgX;
            pid.setSetpoint(degreeSetpoint);
            double pidOutput = pid.getOutput(navx.getAngle());

            SmartDashboard.putNumber("pointToTarget/pidOutput", pidOutput);
            SmartDashboard.putNumber("pointToTarget/avgDegreeDistanceToTarget", avgX);

            
            if (pidOutput > SmartDashboard.getNumber("pointToTarget/pidClamp", 0.8)) {
                pidOutput = 0;
                SmartDashboard.putBoolean("pointToTarget/isClamping", true);
                this.end(false);
            } else {
                SmartDashboard.putBoolean("pointToTarget/isClamping", false);
            }

            drivetrain.setPercent(pidOutput, -pidOutput);
        }
    }

    public void end(boolean isInterrupted) {
        drivetrain.stopMotors();
        limelight.setLeds(false);
    }
}
