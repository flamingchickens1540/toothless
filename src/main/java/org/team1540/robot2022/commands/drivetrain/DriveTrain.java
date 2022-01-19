package org.team1540.robot2022.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.team1540.robot2022.Constants.DriveConstants.Motors;
import org.team1540.robot2022.utils.ChickenTalonFX;
import org.team1540.robot2022.utils.NavX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
    private final ChickenTalonFX driveLFront = new ChickenTalonFX(Motors.leftFront);
    private final ChickenTalonFX driveLRear = new ChickenTalonFX(Motors.leftRear);
    private final ChickenTalonFX driveRFront = new ChickenTalonFX(Motors.rightFront);
    private final ChickenTalonFX driveRRear = new ChickenTalonFX(Motors.rightRear);
    private final ChickenTalonFX[] driveL = { driveLFront, driveLRear };
    private final ChickenTalonFX[] driveR = { driveRFront, driveRRear };
    private final ChickenTalonFX[] driveMotors = { driveLFront, driveLRear, driveRFront, driveRRear };
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
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return driveOdometry.getPoseMeters();

    }

    public void setPercent(double leftPercent, double rightPercent) {
        driveLFront.set(ControlMode.PercentOutput, leftPercent);
        driveRFront.set(ControlMode.PercentOutput, rightPercent);
    }

    public void setNeutralMode(NeutralMode mode) {
        for (ChickenTalonFX motor : driveMotors) {
            motor.setNeutralMode(mode);
        }
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        driveLFront.setVoltage(leftVolts);
        driveRFront.setVoltage(rightVolts);

    }
}
