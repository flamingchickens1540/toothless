package org.team1540.robot2022.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2022.commands.drivetrain.Drivetrain;
import org.team1540.robot2022.utils.Limelight;
import org.team1540.robot2022.utils.NavX;

public class Vision extends SubsystemBase {
    Drivetrain drivetrain;
    NavX navX;
    Limelight limelight;
    Pose2d lastPose;
    double lastDistance;
    double lastRotation;
    boolean isSimulation = false;

    public Vision(Drivetrain drivetrain, NavX navX, Limelight limelight) {
        this.drivetrain = drivetrain;
        this.navX = navX;
        this.limelight = limelight;

        // Force LEDs always on.
        limelight.setLeds(true);

        // Start by assuming the field units are meters (unconfirmed)
        SmartDashboard.putNumber("vision/inchesToFieldRatio", 0.0185);
        SmartDashboard.putBoolean("vision/allowTargetUpdate", false);

        if (isSimulation) {
            SmartDashboard.putNumber("vision/testingX", -4);
            SmartDashboard.putNumber("vision/testingY", -4);
        }
    }

    @Override
    public void periodic() {
        boolean isLimelightAligned = limelight.isTargetAligned();
        SmartDashboard.putBoolean("vision/limelightAligned", isLimelightAligned);
        if (SmartDashboard.getBoolean("vision/allowTargetUpdate", false) && isLimelightAligned && !isSimulation) {
            // lastDistance is the limelight's found distance + distance to center of the hub then made into "field units"
            updateLastTargetPose();
        }
        if (lastPose != null) {
            SmartDashboard.putNumber("vision/estimatedAngle", getNormalizedAngleToTargetDegrees());
        }
    }

    @Override
    public void simulationPeriodic() {
        if (lastPose == null) {
            lastDistance = SmartDashboard.getNumber("vision/distanceInFieldUnits", 0.76264080);
            lastPose = new Pose2d(new Translation2d(8.695, 5.314), new Rotation2d());
            lastRotation = navX.getAngleRadians();

            SmartDashboard.putNumber("vision/testingX", 7.9);
            SmartDashboard.putNumber("vision/testingY", 4.6);
        }
        SmartDashboard.putNumber("vision/lastRotation", lastRotation);
        SmartDashboard.putNumber("vision/poseX", lastPose.getX());
        SmartDashboard.putNumber("vision/poseY", lastPose.getY());
    }

    /**
     * Zeros the last known position to the three/two-ball auto position near the edge of the field.
     */
    public void zeroPosition() {
        lastDistance = 1.21973;
        lastPose = drivetrain.getPose();
        lastRotation = navX.getAngleRadians();
        SmartDashboard.putNumber("vision/lastPoseX", lastPose.getX());
        SmartDashboard.putNumber("vision/lastPoseY", lastPose.getY());
        SmartDashboard.putNumber("vision/lastRotation", lastRotation);
        SmartDashboard.putNumber("vision/lastDistance", lastDistance);
    }

    /**
     * Updates the last pose to the current sensor readings.
     */
    private void updateLastTargetPose() {
        lastDistance = (limelight.getCalculatedDistance() + 31.375) * SmartDashboard.getNumber("vision/inchesToFieldRatio", 0.0185);
        lastPose = drivetrain.getPose();
        lastRotation = navX.getAngleRadians();

        // Push to SmartDashboard
        SmartDashboard.putNumber("vision/lastPoseX", lastPose.getX());
        SmartDashboard.putNumber("vision/lastPoseY", lastPose.getY());
        SmartDashboard.putNumber("vision/lastRotation", lastRotation);
        SmartDashboard.putNumber("vision/lastDistance", lastDistance);
    }

    /**
     * Wraps an angle in radians to be between [0, TWO_PI]
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
     * @return Estimated turn angle to target, in degrees [-180, 180]
     */
    public double getNormalizedAngleToTargetDegrees() {
        return Math.toDegrees(wrapRotationToYaw(getAngleToTargetRadians()));
    }

    /**
     * Calculates target angle in radians, un-normalized to yaw.
     * @return Estimated turn angle to target, in radians [0, TWO_PI]
     */
    public double getAngleToTargetRadians() {
        // double zeroAngleFromFieldY = 1.9386451520732095 - Math.toRadians(90); // y-axis to initialization position, cw
        double zeroAngleFromFieldY = Math.toRadians(18.89); // angle to Y from pointing to hub, when hub is at (8.3,4.16)

        // double zeroAngleFromFieldY = Math.toRadians(45);

        // Get the current pose.
        Translation2d currentPose = drivetrain.getPose().getTranslation();

        SmartDashboard.putNumber("vision/currentPoseX", currentPose.getX());
        SmartDashboard.putNumber("vision/currentPoseY", currentPose.getY());

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

        SmartDashboard.putNumber("vision/translationX", translation.getX());
        SmartDashboard.putNumber("vision/translationY", translation.getY());
        SmartDashboard.putNumber("vision/translationR", translation.getNorm());

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
        double testAngle = Math.atan2(BD.y, BD.x) + Math.PI/2;
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
