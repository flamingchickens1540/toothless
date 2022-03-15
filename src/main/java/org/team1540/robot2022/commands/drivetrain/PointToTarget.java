package org.team1540.robot2022.commands.drivetrain;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2022.utils.Limelight;
import org.team1540.robot2022.utils.MiniPID;
import org.team1540.robot2022.utils.NavX;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.function.DoubleConsumer;

public class PointToTarget extends CommandBase {
    private final Drivetrain drivetrain;
    private final Limelight limelight;
    private final NavX navX;
    private final LinkedList<Vector2d> pastPoses = new LinkedList<>();
    private final MedianFilter medianFilter = new MedianFilter(10);
    private int medianFilterCount = 0;
    private boolean turning = false;

    // A little testing says kP=0.7 and kD=0.4 are fairly strong.
    private final MiniPID pid = new MiniPID(1, 0, 0);

    public PointToTarget(Drivetrain drivetrain, Limelight limelight, NavX navX) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        this.navX = navX;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        double p = SmartDashboard.getNumber("pointToTarget/kP", 0.006);
        double i = SmartDashboard.getNumber("pointToTarget/kI", 0);
        double d = SmartDashboard.getNumber("pointToTarget/kD", 0.015);
        limelight.setLeds(true);
        pid.setPID(p, i, d);
        pid.setSetpoint(0);
    }

    private double getHorizontalDistanceToTarget() {
        return limelight.getTargetAngles().x;
    }

    private double getError(double distanceToTarget) {
        return Math.abs(distanceToTarget) / (limelight.getHorizontalFov() / 2);
    }

    /**
     * Uses a basic average to calculate the degree offset of the target.
     * Waits to fill the averaging array before turning.
     *
     * @param turnFunction a function that takes the output of our finalized angle
     */
    private void calculateWithAverage(DoubleConsumer turnFunction) {
        Vector2d lmAngles = limelight.getTargetAngles();
        pastPoses.add(lmAngles);
        if (pastPoses.size() < 10) {
            return;
        } else if (pastPoses.size() > 10) {
            pastPoses.remove(0);
        }
        double avgX = 0;
        for (Vector2d pose : pastPoses) {
            avgX += pose.x;
        }
        avgX /= pastPoses.size();
        turnFunction.accept(avgX);
    }

    /**
     * Uses a {@link MedianFilter} to calculate the degree offset of the target.
     * Waits to fill the filter's buffer before turning.
     *
     * @param turnFunction a function that takes the output of our finalized angle
     */
    private void calculateWithMedian(DoubleConsumer turnFunction) {
        Vector2d llAngles = limelight.getTargetAngles();
        if (medianFilterCount < 10) {
            medianFilter.calculate(llAngles.x);
            medianFilterCount++;
            return;
        }

        turnFunction.accept(medianFilter.calculate(llAngles.x));
    }

    /**
     * Discards all contour corners that sit below the average vertical value, then finding the average horizontal value
     * of the remaining corners as the point to turn to.
     *
     * @param turnFunction a function that takes the output of our finalized angle
     */
    private void calculateWithCorners(DoubleConsumer turnFunction) {
        double[] cornerCoordinates = limelight.getNetworkTable().getEntry("tcornxy").getDoubleArray(new double[]{});
        ArrayList<Vector2d> cornerPoints = new ArrayList<>(cornerCoordinates.length / 2);
        for (int i = 0; i < cornerCoordinates.length; i += 2) {
            cornerPoints.add(new Vector2d(cornerCoordinates[i], cornerCoordinates[i + 1]));
        }

        double cornerYSum = 0;
        for (Vector2d point : cornerPoints) {
            cornerYSum += point.y;
        }
        double cornerYAvg = cornerYSum / cornerPoints.size();

        for (int i = 0; i < cornerPoints.size(); i++) {
            if (cornerPoints.get(i).y > cornerYAvg) {
                cornerPoints.remove(i);
                i--;
            }
        }

        // TODO: This might be better as a median, depends on how good I get the pipeline to work.
        double correctedCornerXSum = 0;
        for (Vector2d point : cornerPoints) {
            correctedCornerXSum += point.x;
        }
        double correctedCornerXAvg = correctedCornerXSum / cornerPoints.size();

        // Steps to calculate on-screen coordinate offsets as angles, as given in the Limelight docs.
        double normalizedCornerXAvg = (2.0 / limelight.getResolution().x) * (correctedCornerXAvg - (limelight.getResolution().x / 2 - 0.5));
        double viewportCornerXAvg = Math.tan(limelight.getHorizontalFov() / 2) * normalizedCornerXAvg;
        double degreeOffsetCornerXAvg = Math.toDegrees(Math.atan2(viewportCornerXAvg, 1));
        SmartDashboard.putNumber("pointToTarget/corner/correctedCornerX", correctedCornerXAvg);
        SmartDashboard.putNumber("pointToTarget/corner/offsetNormalizedX", normalizedCornerXAvg);
        SmartDashboard.putNumber("pointToTarget/corner/offsetAvg", degreeOffsetCornerXAvg);
        turnFunction.accept(degreeOffsetCornerXAvg);
    }

    /**
     * Executes turning to the target using the Limelight as the primary sensor to determine whether we have turned enough.
     *
     * @param angleXOffset the offset in degrees we still need to turn to reach the target
     */
    private void turnWithLimelight(double angleXOffset) {
        if (Math.abs(angleXOffset) > SmartDashboard.getNumber("pointToTarget/targetDeadzoneDegrees", 2)) {

            double distanceToTarget = getHorizontalDistanceToTarget();
            double pidOutput = pid.getOutput(getError(angleXOffset));
            double multiplier = angleXOffset > 0 ? 1 : -1;

            SmartDashboard.putNumber("pointToTarget/pidOutput", pidOutput);
            SmartDashboard.putNumber("pointToTarget/degreeDistanceToTarget", distanceToTarget);

            pidOutput = clampPID(pidOutput);
            double valueL = multiplier * -pidOutput;
            double valueR = multiplier * pidOutput;
            drivetrain.setPercent(valueL, valueR);
        }
    }

    /**
     * Executes turning to the target using the NavX as the primary sensor to determine whether we have turned enough.
     *
     * @param startAngleXOffset the starting setpoint of where the NavX should attempt to turn to
     */
    private void turnWithNavX(double startAngleXOffset) {
        if (!turning) {
            turning = true;
            double degreeSetpoint = navX.getAngle() + startAngleXOffset;
            pid.setSetpoint(degreeSetpoint);
        }

        double pidOutput = clampPID(pid.getOutput(navX.getAngle()));
        drivetrain.setPercent(pidOutput, -pidOutput);
    }

    /**
     * Clamps the given PID output based on SmartDashboard values.
     *
     * @param pidOutput the value to clamp
     * @return the clamped value
     */
    private double clampPID(double pidOutput) {
        if (pidOutput > SmartDashboard.getNumber("pointToTarget/pidClamp", 0.8)) {
            SmartDashboard.putBoolean("pointToTarget/isClamping", true);
            this.end(false);
            return 0;
        } else {
            SmartDashboard.putBoolean("pointToTarget/isClamping", false);
            return pidOutput;
        }
    }

    public void execute() {
        calculateWithCorners(this::turnWithLimelight);
    }

    public void end(boolean isInterrupted) {
        drivetrain.stopMotors();
        limelight.setLeds(false);
        medianFilterCount = 0;
    }
}
