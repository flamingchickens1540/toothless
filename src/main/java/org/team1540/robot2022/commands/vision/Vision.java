package org.team1540.robot2022.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2022.commands.drivetrain.Drivetrain;
import org.team1540.robot2022.utils.AverageFilter;
import org.team1540.robot2022.utils.ChickenSmartDashboard;
import org.team1540.robot2022.utils.Limelight;
import org.team1540.robot2022.utils.NavX;

import java.util.ArrayList;

public class Vision extends SubsystemBase {
    Drivetrain drivetrain;
    NavX navX;
    Limelight limelight;
    Pose2d lastPose = new Pose2d();
    double lastDistance;
    double lastRotation;
    boolean isSimulation = false;

    AverageFilter cornerAverageX = new AverageFilter(10);
    AverageFilter cornerAverageY = new AverageFilter(10);

    public Vision(Drivetrain drivetrain, NavX navX, Limelight limelight) {
        this.drivetrain = drivetrain;
        this.navX = navX;
        this.limelight = limelight;

        // Force LEDs always on.
        limelight.setLeds(true);

        // Start by assuming the field units are meters (unconfirmed)`
        ChickenSmartDashboard.putDebugNumber("vision/inchesToFieldRatio", 0.0185);
        ChickenSmartDashboard.putDebugBoolean("vision/allowTargetUpdate", true);

        if (isSimulation) {
            ChickenSmartDashboard.putDebugNumber("vision/testingX", -4);
            ChickenSmartDashboard.putDebugNumber("vision/testingY", -4);
        }
    }

    @Override
    public void periodic() {
        boolean isLimelightAligned = limelight.isTargetAligned();
        ChickenSmartDashboard.putDebugBoolean("vision/limelightAligned", isLimelightAligned);
        if (SmartDashboard.getBoolean("vision/allowTargetUpdate", false) && isLimelightAligned && !isSimulation) {
            // lastDistance is the limelight's found distance + distance to center of the hub then made into "field units"
            updateLastTargetPose();
        }
        if (lastPose != null) {
            ChickenSmartDashboard.putDebugNumber("vision/estimatedAngle", getNormalizedAngleToTargetDegrees());
        }
        if (limelight.isTargetFound()) {
            calculateCornerAverages();
        } else {
            cornerAverageX.clear();
            cornerAverageY.clear();
        }
        ChickenSmartDashboard.putDebugNumber("vision/estimatedDistance", getCornerCalculatedDistance());
    }


    @Override
    public void simulationPeriodic() {
        if (lastPose == null) {
            lastDistance = SmartDashboard.getNumber("vision/distanceInFieldUnits", 0.76264080);
            lastPose = new Pose2d(new Translation2d(8.695, 5.314), new Rotation2d());
            lastRotation = navX.getAngleRadians();

            ChickenSmartDashboard.putDebugNumber("vision/testingX", 7.9);
            ChickenSmartDashboard.putDebugNumber("vision/testingY", 4.6);
        }
        ChickenSmartDashboard.putDebugNumber("vision/lastRotation", lastRotation);
        ChickenSmartDashboard.putDebugNumber("vision/poseX", lastPose.getX());
        ChickenSmartDashboard.putDebugNumber("vision/poseY", lastPose.getY());
    }

    /**
     * Determines averages of all limelight corners by discarding statistical outliers beneath the average vertical value
     * of all corners, then adding the average x and y values of those corners (normalized to degrees from the center)
     * into averaging filters set to 10 periods.
     */
    private void calculateCornerAverages() {
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

        double correctedCornerXSum = 0;
        double correctedCornerYSum = 0;
        for (Vector2d point : cornerPoints) {
            correctedCornerXSum += point.x;
            correctedCornerYSum += point.y;
        }
        double correctedCornerXAvg = correctedCornerXSum / cornerPoints.size();
        double correctedCornerYAvg = correctedCornerYSum / cornerPoints.size();

        // Steps to calculate on-screen coordinate offsets as angles, as given in the Limelight docs.
        double normalizedCornerXAvg = (2.0 / limelight.getResolution().x) * (correctedCornerXAvg - (limelight.getResolution().x / 2 - 0.5));
        double normalizedCornerYAvg = (2.0 / limelight.getResolution().y) * (correctedCornerYAvg - (limelight.getResolution().y / 2 - 0.5));

        double viewportCornerXAvg = Math.tan(limelight.getHorizontalFov() / 2) * normalizedCornerXAvg;
        double viewportCornerYAvg = Math.tan(limelight.getVerticalFov() / 2) * normalizedCornerYAvg;

        double degreeOffsetCornerXAvg = Math.toDegrees(Math.atan2(viewportCornerXAvg, 1));
        double degreeOffsetCornerYAvg = Math.toDegrees(Math.atan2(viewportCornerYAvg, 1));

        // SmartDashboard.putNumber("pointToTarget/corner/correctedCornerX", correctedCornerXAvg);
        // SmartDashboard.putNumber("pointToTarget/corner/offsetNormalizedX", normalizedCornerXAvg);
        ChickenSmartDashboard.putDebugNumber("pointToTarget/corner/offsetXAvg", degreeOffsetCornerXAvg);
        ChickenSmartDashboard.putDebugNumber("pointToTarget/corner/offsetYAvg", degreeOffsetCornerYAvg);

        cornerAverageX.add(degreeOffsetCornerXAvg);
        cornerAverageY.add(-degreeOffsetCornerYAvg);
    }

    /**
     * Gets the averages for the limelight's found corners, as calculated in {@link #calculateCornerAverages()}.
     *
     * @return {@link Vector2d} the average degree offset for x and y coordinates
     */
    public Vector2d getCornerAverages() {
        return new Vector2d(cornerAverageX.getAverage(), cornerAverageY.getAverage());
    }

    /**
     * Gets the current calculated distance of the limelight to the base of the hub.
     *
     * @return the distance in inches
     */
    public double getCalculatedDistance() {
        double theta = Math.toRadians(limelight.getTargetAngles().y) + limelight.limelightAngle;
        double actualHeight = limelight.targetHeight - limelight.limelightHeight;
        return 39.37007874 * actualHeight / Math.tan(theta);
    }

    public double getCornerCalculatedDistance() {
        double theta = Math.toRadians(getCornerAverages().y) + limelight.limelightAngle;
        double actualHeight = limelight.targetHeight - limelight.limelightHeight;
        return 39.37007874 * actualHeight / Math.tan(theta);
    }

    /**
     * Zeros the last known position to the three/two-ball auto position near the edge of the field.
     */
    public void zeroPosition() {
        lastDistance = 1.21973;
        lastPose = drivetrain.getPose();
        lastRotation = navX.getAngleRadians();
        ChickenSmartDashboard.putDebugNumber("vision/lastPoseX", lastPose.getX());
        ChickenSmartDashboard.putDebugNumber("vision/lastPoseY", lastPose.getY());
        ChickenSmartDashboard.putDebugNumber("vision/lastRotation", lastRotation);
        ChickenSmartDashboard.putDebugNumber("vision/lastDistance", lastDistance);
    }

    /**
     * Updates the last pose to the current sensor readings.
     */
    private void updateLastTargetPose() {
        lastDistance = (getCalculatedDistance() + 31.375) * SmartDashboard.getNumber("vision/inchesToFieldRatio", 0.0185);
        lastPose = drivetrain.getPose();
        lastRotation = navX.getAngleRadians();

        // Push to SmartDashboard
        ChickenSmartDashboard.putDebugNumber("vision/lastPoseX", lastPose.getX());
        ChickenSmartDashboard.putDebugNumber("vision/lastPoseY", lastPose.getY());
        ChickenSmartDashboard.putDebugNumber("vision/lastRotation", lastRotation);
        ChickenSmartDashboard.putDebugNumber("vision/lastDistance", lastDistance);
    }

    /**
     * Wraps an angle in radians to be between [0, TWO_PI]
     *
     * @param rotation Radian angle to convert
     * @return Converted angle in radians, [0, TWO_PI]
     */
    private double wrapRotation(double rotation) {
        double scaled = rotation % (2 * Math.PI);
        if (scaled < 0) {
            scaled = 2 * Math.PI + scaled;
        }
        return scaled;
    }

    /**
     * Wraps an angle in radians to be between [-PI, PI]
     *
     * @param radians Radian angle to convert
     * @return Converted angle in radians, [-PI, PI]
     */
    private double wrapRotationToYaw(double radians) {
        if (radians > Math.PI) {
            return radians - 2 * Math.PI;
        }
        return radians;
    }

    /**
     * Calculates target angle in degrees, normalized to yaw.
     *
     * @return Estimated turn angle to target, in degrees [-180, 180]
     */
    public double getNormalizedAngleToTargetDegrees() {
//        return Math.toDegrees(wrapRotationToYaw(getAngleToTargetRadians()));
        return -90.0;
    }

    /**
     * Calculates target angle in radians, un-normalized to yaw.
     *
     * @return Estimated turn angle to target, in radians [0, TWO_PI]
     */
    public double getAngleToTargetRadians() {
        // double zeroAngleFromFieldY = 1.9386451520732095 - Math.toRadians(90); // y-axis to initialization position, cw
        double zeroAngleFromFieldY = Math.toRadians(18.89); // angle to Y from pointing to hub, when hub is at (8.3,4.16)

        // double zeroAngleFromFieldY = Math.toRadians(45);

        // Get the current pose.
        Translation2d currentPose = drivetrain.getPose().getTranslation();

        ChickenSmartDashboard.putDebugNumber("vision/currentPoseX", currentPose.getX());
        ChickenSmartDashboard.putDebugNumber("vision/currentPoseY", currentPose.getY());

        // This should be removed
        if (isSimulation) {
            currentPose = new Translation2d(SmartDashboard.getNumber("vision/testingX", -5), SmartDashboard.getNumber("vision/testingY", 0));
        }

        // The current angle gets the navX yaw angle, and adds our original offset from positive x-axis.
        double currentAngle = wrapRotation(zeroAngleFromFieldY);
        // The last angle gets the last recorded navX yaw angle, and adds our original offset from positive x-axis.
        double lastAngle = wrapRotation(lastRotation + zeroAngleFromFieldY + 2 * Math.PI);

//        SmartDashboard.putNumber("sim/1.1_currentAngle", Math.toDegrees(currentAngle));
//        SmartDashboard.putNumber("sim/1.2_lastAngle", Math.toDegrees(lastAngle));

        // Gets the vector between our last pose and our new pose.
        Translation2d translation = currentPose.minus(lastPose.getTranslation());

        ChickenSmartDashboard.putDebugNumber("vision/translationX", translation.getX());
        ChickenSmartDashboard.putDebugNumber("vision/translationY", translation.getY());
        ChickenSmartDashboard.putDebugNumber("vision/translationR", translation.getNorm());

        Vector2d BD = new Vector2d(translation.getX(), translation.getY());

//        SmartDashboard.putNumber("sim/1_magBD", BD.magnitude());

        // The magnitude of the vector from our last pose to the hub.
        double magAB = lastDistance;

//        SmartDashboard.putNumber("sim/2_magAB", magAB);

        // Unit vector perpendicular to the line through last position and hub (AB)
        Vector2d unitBF = new Vector2d(0, -1);

        unitBF.rotate(-(Math.toDegrees(wrapRotation(lastAngle + Math.toRadians(90)))));

//        SmartDashboard.putNumber("sim/3.1_lastAngle", Math.toDegrees(lastAngle));
//        SmartDashboard.putNumber("sim/3_dirBF", Math.toDegrees(wrapRotation(lastAngle + Math.toRadians(90))));
        double magAC = BD.dot(unitBF);
//        SmartDashboard.putNumber("sim/4_magAC", magAC);

        if (magAC > 0) {
            unitBF.rotate(180);
        }

        Vector2d AC = new Vector2d(unitBF.x * magAC, unitBF.y * magAC);
//        SmartDashboard.putNumber("sim/5_xAC", AC.x);
        Vector2d unitBA = new Vector2d(0, -1);
        unitBA.rotate(-Math.toDegrees(lastAngle));
//        SmartDashboard.putNumber("sim/6_dirAB",  Math.toDegrees(lastAngle));
        double magFD = BD.dot(unitBA);
//        SmartDashboard.putNumber("sim/7_magFD", magFD);
        double magCD = magAB - magFD;
        Vector2d CD = new Vector2d(unitBF.x * magCD, unitBF.y * magCD);
//        SmartDashboard.putNumber("sim/8.1_magCD", magCD);
        double theta = Math.atan2(AC.magnitude(), CD.magnitude());
//        SmartDashboard.putNumber("sim/8_thetaDegrees", Math.toDegrees(theta));
        double angleToTurnTo = currentAngle + theta - zeroAngleFromFieldY;

        // testAngle helps to test what quadrant our robot moved to, to properly assign neg/pos turning values.
        double testAngle = Math.atan2(BD.y, BD.x) + Math.PI / 2;
//        SmartDashboard.putNumber("sim/9_testAngle", Math.toDegrees(testAngle));

        if (testAngle + currentAngle < 0 || testAngle + currentAngle > Math.PI) {
            angleToTurnTo = Math.PI * 2 - angleToTurnTo;
        }

        if (magCD < 0) {
            angleToTurnTo = Math.PI - angleToTurnTo;
        }

        // Returns in radians, wrapped to be within [0, TWO_PI]
        return wrapRotation(angleToTurnTo + navX.getAngleRadians());
    }
}
