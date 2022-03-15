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
    boolean isSimulation = true;

    public Vision(Drivetrain drivetrain, NavX navX, Limelight limelight) {
        this.drivetrain = drivetrain;
        this.navX = navX;
        this.limelight = limelight;
        
        if (isSimulation) {
            SmartDashboard.putNumber("vision/testingX", -4);
            SmartDashboard.putNumber("vision/testingY", -4);
        }
    }

    @Override
    public void periodic() {
        if (limelight.isTargetFound()) {
            lastDistance = limelight.getCalculatedDistance();
            lastPose = drivetrain.getPose();
            lastRotation = navX.getAngleRadians();
            SmartDashboard.putNumber("vision/poseX", lastPose.getX());
            SmartDashboard.putNumber("vision/poseY", lastPose.getY());
        }
        if (lastPose != null) {
            SmartDashboard.putNumber("vision/estimatedAngle", Math.toDegrees(getAngleToTarget()));
        }
    }

    @Override
    public void simulationPeriodic() {
        if (lastPose == null) {
            lastDistance = SmartDashboard.getNumber("vision/distanceInFieldUnits", 1.486479);
            lastPose = new Pose2d(new Translation2d(8.695, 5.314), new Rotation2d());
            lastRotation = navX.getAngleRadians();

            SmartDashboard.putNumber("vision/testingX", 13);
            SmartDashboard.putNumber("vision/testingY", 5);
        }
        SmartDashboard.putNumber("vision/poseX", lastPose.getX());
        SmartDashboard.putNumber("vision/poseY", lastPose.getY());
    }

    private double wrapRotation(double rotation) {
        if (rotation > Math.PI) {
            return rotation - Math.PI * 2;
        } else if (rotation < -Math.PI) {
            return rotation + Math.PI * 2;
        }
        return rotation;
    }

    public double getAngleToTarget() {
        // double zeroAngleFromFieldY = 1.9386451520732095 - Math.toRadians(90); // y-axis to initialization position, cw
        double zeroAngleFromFieldY = Math.toRadians(27.8752288); // testing y-axis to initialization position, cw

        // double zeroAngleFromFieldY = Math.toRadians(45);

        // Get the current pose.
        Translation2d currentPose = drivetrain.getPose().getTranslation();

        if (isSimulation) {
            currentPose = new Translation2d(SmartDashboard.getNumber("vision/testingX", -5), SmartDashboard.getNumber("vision/testingY", 0));
        }

        // The current angle gets the navX yaw angle, and adds our original offset from positive x-axis.
        double currentAngle = wrapRotation(navX.getYawRadians() + zeroAngleFromFieldY);
        // The last angle gets the last recorded navX yaw angle, and adds our original offset from positive x-axis.
        double lastAngle = wrapRotation(lastRotation + zeroAngleFromFieldY);

        // Gets the vector between our last pose and our new pose.
        Translation2d translation = lastPose.getTranslation().minus(currentPose);
        
        Vector2d BD = new Vector2d(translation.getX(), translation.getY());

        SmartDashboard.putNumber("sim/1_magBD", BD.magnitude());

        // The magnitude of the vector from our last pose to the hub.
        double magAB = lastDistance;

        SmartDashboard.putNumber("sim/2_magAB", magAB);

        // Unit vector perpendicular to the line through last position and hub (AB)
        Vector2d unitBF = new Vector2d(0, -1);
        unitBF.rotate(-(Math.toDegrees(lastAngle - Math.toRadians(90))));
        
        SmartDashboard.putNumber("sim/3.1_lastAngle", Math.toDegrees(lastAngle));
        SmartDashboard.putNumber("sim/3_dirBF", Math.toDegrees(lastAngle - Math.toRadians(90)));

        double magAC = BD.dot(unitBF);
        SmartDashboard.putNumber("sim/4_magAC", magAC);

        Vector2d AC = new Vector2d(unitBF.x * magAC, unitBF.y * magAC);

        SmartDashboard.putNumber("sim/5_xAC", AC.x);

        Vector2d unitBA = new Vector2d(0, -1);
        unitBA.rotate(-Math.toDegrees(lastAngle));

        SmartDashboard.putNumber("sim/6_dirAB",  Math.toDegrees(lastAngle));

        double magFD = BD.dot(unitBA);

        SmartDashboard.putNumber("sim/7_magFD", magFD);

        double magCD = magAB + magFD;
        Vector2d CD = new Vector2d(unitBF.x * magCD, unitBF.y * magCD);

        double theta = Math.atan2(AC.magnitude(), CD.magnitude());
        double angleToTurnTo = currentAngle + theta - zeroAngleFromFieldY;

        // Should return in radians
        return angleToTurnTo;
    }
}
