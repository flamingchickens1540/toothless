package org.team1540.robot2022.commands.drivetrain;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2022.commands.vision.Vision;
import org.team1540.robot2022.utils.*;

import java.util.LinkedList;
import java.util.function.DoubleConsumer;

public class PointToTarget extends CommandBase {
    private final Drivetrain drivetrain;
    private final Vision vision;
    private final Limelight limelight;
    private final NavX navX;
    private final ChickenXboxController controller;
    private final LinkedList<Vector2d> pastPoses = new LinkedList<>();
    private final MedianFilter medianFilter = new MedianFilter(10);
    private final AverageFilter averageFilter = new AverageFilter(10);
    private int medianFilterCount = 0;
    private boolean turning = false;

    NetworkTable mediapipeTable;

    // A little testing says kP=0.7 and kD=0.4 are fairly strong.
    private final MiniPID pid = new MiniPID(1, 0, 0);
    private final MiniPID pidNavX = new MiniPID(0, 0, 0);
    private boolean isFinished = false;

    public PointToTarget(Drivetrain drivetrain, Vision vision, Limelight limelight, NavX navX, ChickenXboxController controller) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.limelight = limelight;
        this.navX = navX;
        this.controller = controller;

        this.mediapipeTable = NetworkTableInstance.getDefault().getTable("mediapipe");

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        double p = SmartDashboard.getNumber("pointToTarget/kP", 0.008);
        double i = SmartDashboard.getNumber("pointToTarget/kI", 0);
        double d = SmartDashboard.getNumber("pointToTarget/kD", 0.05);

        double pX = SmartDashboard.getNumber("pointToTarget/navX_kP", 0);
        double dX = SmartDashboard.getNumber("pointToTarget/navX_kD", 0);

        pidNavX.setPID(pX, 0, dX);
        pidNavX.setSetpoint(0);

        pid.setPID(p, i, d);
        pid.setSetpoint(0);

        limelight.setLeds(true);
        ChickenSmartDashboard.putDebugBoolean("pointToTarget/turningWithLimelight", true);
        System.out.println("PTT Initialized");
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

    private void calculateAndTurnWithMediapipePose() {

        double offsetX = mediapipeTable.getEntry("noseX").getDouble(0);
        System.out.println("Nose X Offset: " + offsetX);

        if (Math.abs(offsetX) > SmartDashboard.getNumber("pointToTarget/mpNoseXDeadzone", 20)) {

            double distanceToTarget = getMediapipeDistance();
            double pidOutput = pid.getOutput(getError(offsetX));
            double multiplier = offsetX > 0 ? 1 : -1;

            ChickenSmartDashboard.putDebugNumber("pointToTarget/mpPidOutput", pidOutput);
            ChickenSmartDashboard.putDebugNumber("pointToTarget/mpDistanceToTarget", distanceToTarget);

            pidOutput = clampPID(pidOutput);
            double valueL = multiplier * -pidOutput;
            double valueR = multiplier * pidOutput;
            drivetrain.setPercent(valueL, valueR);
        }
    }

    private double getMediapipeDistance() {
        return mediapipeTable.getEntry("headWidth").getDouble(0);
    }

    /**
     * Discards all contour corners that sit below the average vertical value, then finding the average horizontal value
     * of the remaining corners as the point to turn to.
     *
     * @param turnFunction a function that takes the output of our finalized angle
     */
    private void calculateWithCorners(DoubleConsumer turnFunction) {
        turnFunction.accept(vision.getCornerAverages().x);
    }

    /**
     * Executes turning to the target using the Limelight as the primary sensor to determine whether we have turned enough.
     *
     * @param angleXOffset the offset in degrees we still need to turn to reach the target
     */
    private void turnWithLimelight(double angleXOffset) {
        System.out.println(angleXOffset);
        if (Math.abs(angleXOffset) > SmartDashboard.getNumber("pointToTarget/targetDeadzoneDegrees", 5)) {

            double distanceToTarget = getHorizontalDistanceToTarget();
            double pidOutput = pid.getOutput(getError(angleXOffset));
            double multiplier = angleXOffset > 0 ? 1 : -1;

            ChickenSmartDashboard.putDebugNumber("pointToTarget/pidOutput", pidOutput);
            ChickenSmartDashboard.putDebugNumber("pointToTarget/degreeDistanceToTarget", distanceToTarget);

            pidOutput = clampPID(pidOutput);
            double valueL = multiplier * -pidOutput;
            double valueR = multiplier * pidOutput;
            drivetrain.setPercent(valueL, valueR);
        } else {
            System.out.println("Ending TWL");
            this.isFinished = true;
        }
    }

    /**
     * Executes turning to the target using the NavX as the primary sensor to determine whether we have turned enough.
     *
     * @param angleXOffset the starting setpoint of where the NavX should attempt to turn to
     */
    private void turnWithNavX(double angleXOffset) {
        double multiplier = angleXOffset > 0 ? 1 : -1;

        double pidOut = pidNavX.getOutput(Math.abs(angleXOffset / 180.0));
        double pidOutput = clampPID(pidOut);
        pidOutput = clampPID(pidOutput);
        double valueL = multiplier * -pidOutput;
        double valueR = multiplier * pidOutput;

        drivetrain.setPercent(valueL, valueR);
    }

    /**
     * Executes turning to the target using {@link #turnWithLimelight} if the target is found,
     * otherwise using {@link #turnWithNavX} if we need to use the Vision-estimated angle.
     */
    private void calculateAndTurnWithVision() {
        if (limelight.isTargetFound()) {
            ChickenSmartDashboard.putDebugBoolean("pointToTarget/turningWithLimelight", true);
            NetworkTableInstance.getDefault().flush();
            pid.setSetpoint(0);
            calculateWithCorners(this::turnWithLimelight);
        } else {
            System.out.println("TURNING, NO LIMELIGHT TARGET FOUND");
            if (controller != null) {
                controller.setRumble(0.1);
            }
            ChickenSmartDashboard.putDebugBoolean("pointToTarget/turningWithLimelight", false);
            NetworkTableInstance.getDefault().flush();
            turnWithNavX(vision.getNormalizedAngleToTargetDegrees());
        }
    }

    /**
     * Clamps the given PID output based on SmartDashboard values.
     *
     * @param pidOutput the value to clamp
     * @return the clamped value
     */
    private double clampPID(double pidOutput) {
        if (pidOutput > SmartDashboard.getNumber("pointToTarget/pidClamp", 0.8)) {
            ChickenSmartDashboard.putDebugBoolean("pointToTarget/isClamping", true);
            this.end(false);
            return 0;
        } else {
            ChickenSmartDashboard.putDebugBoolean("pointToTarget/isClamping", false);
            return pidOutput;
        }
    }

    public void execute() {
        double p = SmartDashboard.getNumber("pointToTarget/kP", 0.006);
        double i = SmartDashboard.getNumber("pointToTarget/kI", 0);
        double d = SmartDashboard.getNumber("pointToTarget/kD", 0.015);
        pid.setPID(p, i, d);

        double pX = SmartDashboard.getNumber("pointToTarget/navX_kP", 0);
        double dX = SmartDashboard.getNumber("pointToTarget/navX_kD", 0);
        pidNavX.setPID(pX, 0, dX);

//        calculateWithCorners(this::turnWithLimelight);
//        calculateAndTurnWithVision();
        calculateAndTurnWithMediapipePose();
    }

    public boolean isFinished() {
        return this.isFinished;
    }

    @Override
    public void end(boolean isInterrupted) {
        turning = false;
        drivetrain.stopMotors();
//        limelight.setLeds(false);
        medianFilterCount = 0;
        isFinished = false;
        if (controller != null) {
            controller.setRumble(0);
        }
    }
}
