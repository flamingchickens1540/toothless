package org.team1540.robot2022.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2022.Constants.DriveConstants.Motors;
import org.team1540.robot2022.utils.ChickenTalonFX;
import org.team1540.robot2022.utils.NavX;

public class DriveTrain extends SubsystemBase {
    private final ChickenTalonFX driveLFront = new ChickenTalonFX(Motors.leftFront);
    private final ChickenTalonFX driveLRear = new ChickenTalonFX(Motors.leftRear);
    private final ChickenTalonFX driveRFront = new ChickenTalonFX(Motors.rightFront);
    private final ChickenTalonFX driveRRear = new ChickenTalonFX(Motors.rightRear);
    private final ChickenTalonFX[] driveL = {driveLFront, driveLRear};
    private final ChickenTalonFX[] driveR = {driveRFront, driveRRear};
    private final ChickenTalonFX[] driveMotors = {driveLFront, driveLRear, driveRFront, driveRRear};
    private final NavX navx;
    private final DifferentialDriveOdometry driveOdometry;

    public DriveTrain(NeutralMode brakeType, NavX navx) {
        
        driveOdometry = new DifferentialDriveOdometry(navx.getRotation2d());
        this.navx = navx;
        for (ChickenTalonFX motor : driveMotors) {
            motor.configFactoryDefault();
            motor.setNeutralMode(brakeType);
        }
        // Set configuration for left motors
        for (ChickenTalonFX motor : driveL) {
            motor.setInverted(false);
        }
        // Set configuration for right motors
        for (ChickenTalonFX motor : driveR) {
            motor.setInverted(true);
        }
        driveLRear.follow(driveLFront);
        driveRRear.follow(driveRFront);
    }

    @Override
    public void periodic() {
        driveOdometry.update(
                navx.getRotation2d(),
                driveLFront.getDistanceMeters(),
                driveRFront.getDistanceMeters());

        SmartDashboard.putNumber("driveTrain/leftEncoder", driveLFront.getDistanceMeters());
        SmartDashboard.putNumber("driveTrain/rightEncoder", driveRFront.getDistanceMeters());
    }

    public void resetEncoders() {
        for (ChickenTalonFX motor : driveMotors) {
            motor.setSelectedSensorPosition(0);
        }
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                driveLFront.getRateMetersPerSecond(),
                driveRFront.getRateMetersPerSecond());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        driveOdometry.resetPosition(pose, navx.getRotation2d());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return driveOdometry.getPoseMeters();
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        navx.reset();
    }

    public void setPercent(double leftPercent, double rightPercent) {
        driveLFront.set(ControlMode.PercentOutput, leftPercent);
        driveRFront.set(ControlMode.PercentOutput, rightPercent);
    }

    public void stopMotors() {
        this.setPercent(0, 0);
    }

    public Command commandStop() {
        return new InstantCommand(this::stopMotors, this);
    }

    public void setNeutralMode(NeutralMode mode) {
        for (ChickenTalonFX motor : driveMotors) {
            motor.setNeutralMode(mode);
        }
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        SmartDashboard.putNumber("driveTrain/auto/leftVolts", leftVolts);
        SmartDashboard.putNumber("driveTrain/auto/rightVolts", rightVolts);
        driveLFront.setVoltage(leftVolts);
        driveRFront.setVoltage(rightVolts);
    }
}
