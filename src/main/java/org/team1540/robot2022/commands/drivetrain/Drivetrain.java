package org.team1540.robot2022.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2022.Constants;
import org.team1540.robot2022.Constants.DriveConstants.Motors;
import org.team1540.robot2022.utils.ChickenTalonFX;
import org.team1540.robot2022.utils.NavX;

public class Drivetrain extends SubsystemBase {
    private final ChickenTalonFX driveLFront = new ChickenTalonFX(Motors.LEFT_FRONT);
    private final ChickenTalonFX driveLRear = new ChickenTalonFX(Motors.LEFT_REAR);
    private final ChickenTalonFX driveRFront = new ChickenTalonFX(Motors.RIGHT_FRONT);
    private final ChickenTalonFX driveRRear = new ChickenTalonFX(Motors.RIGHT_REAR);
    private final ChickenTalonFX[] driveL = {driveLFront, driveLRear};
    private final ChickenTalonFX[] driveR = {driveRFront, driveRRear};
    private final ChickenTalonFX[] driveMotors = {driveLFront, driveLRear, driveRFront, driveRRear};
    private final NavX navx;
    private final DifferentialDriveOdometry driveOdometry;

    private final SimpleMotorFeedforward ffLeft = new SimpleMotorFeedforward(Constants.DriveConstants.KS_VOLTS, Constants.DriveConstants.KV_VOLT_SECONDS_PER_METER, Constants.DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER);
    private final SimpleMotorFeedforward ffRight = new SimpleMotorFeedforward(Constants.DriveConstants.KS_VOLTS, Constants.DriveConstants.KV_VOLT_SECONDS_PER_METER, Constants.DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER);

    public final Field2d dashboardField = new Field2d();

    public static final double motorToMPS = 26.0349916751;

    public Drivetrain(NeutralMode brakeType, NavX navx) {
        driveOdometry = new DifferentialDriveOdometry(navx.getRotation2d());
        this.navx = navx;

        for (ChickenTalonFX motor : driveMotors) {
            motor.configFactoryDefault();
            motor.setNeutralMode(brakeType);
        }
        for (ChickenTalonFX motor : driveL) {
            motor.setInverted(false);
        }
        for (ChickenTalonFX motor : driveR) {
            motor.setInverted(true);
        }
        driveLRear.follow(driveLFront);
        driveRRear.follow(driveRFront);

        updatePIDs();
        SmartDashboard.putData("drivetrain/field", dashboardField);
    }

    @Override
    public void periodic() {
        driveOdometry.update(
                navx.getRotation2d(),
                driveLFront.getDistanceMeters(),
                driveRFront.getDistanceMeters());

        SmartDashboard.putNumber("drivetrain/leftEncoderMeters", driveLFront.getDistanceMeters());
        SmartDashboard.putNumber("drivetrain/rightEncoderMeters", driveRFront.getDistanceMeters());
        SmartDashboard.putNumber("drivetrain/PID/errorL", driveLFront.getClosedLoopError());
        SmartDashboard.putNumber("drivetrain/PID/errorR", driveRFront.getClosedLoopError());
        dashboardField.setRobotPose(driveOdometry.getPoseMeters());
    }

    /**
     * Sets the velocity of the motors in meters per second
     *
     * @param leftVelocity  The velocity to set the right motors to
     * @param rightVelocity The velocity to set the left motors to
     */
    public void setVelocity(double leftVelocity, double rightVelocity) {
        driveLFront.set(ControlMode.Velocity, leftVelocity * motorToMPS);
        driveRFront.set(ControlMode.Velocity, rightVelocity * motorToMPS);
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

    /**
     * Sets the percent output for the drivetrain motors
     *
     * @param leftPercent  the percent to run the left side at
     * @param rightPercent the percent to run the right side at
     */
    public void setPercent(double leftPercent, double rightPercent) {
        driveLFront.set(ControlMode.PercentOutput, leftPercent);
        driveRFront.set(ControlMode.PercentOutput, rightPercent);
    }

    /**
     * Stops both sides of the drivetrain
     */
    public void stopMotors() {
        this.setPercent(0, 0);
    }

    /**
     * Returns an {@link InstantCommand} to stop the drivetrain
     *
     * @return the InstantCommand
     */
    public Command commandStop() {
        return new InstantCommand(this::stopMotors, this);
    }

    /**
     * Sets the NeutralMode for the drivetrain (either coast or brake)
     *
     * @param mode The mode to set the wheels to
     */
    public void setNeutralMode(NeutralMode mode) {
        for (ChickenTalonFX motor : driveMotors) {
            motor.setNeutralMode(mode);
        }
    }

    /**
     * Sets the voltage for both sides of the drivetrain
     *
     * @param leftVolts  The voltage for the left side
     * @param rightVolts The voltage for the right side
     */
    public void setVolts(double leftVolts, double rightVolts) {
        driveLFront.setVoltage(leftVolts);
        driveRFront.setVoltage(rightVolts);
    }

    /**
     * Sets motor voltages from velocity setpoints
     *
     * @param leftVelocity  left motor velocity RPM setpoint
     * @param rightVelocity right motor velocity RPM setpoint
     */
    public void setFFVelocity(double leftVelocity, double rightVelocity) {
        SmartDashboard.putNumber("drivetrain/feedforward/leftVelocity", leftVelocity);
        SmartDashboard.putNumber("drivetrain/feedforward/rightVelocity", rightVelocity);

        double leftVolts = ffLeft.calculate(leftVelocity);
        double rightVolts = ffRight.calculate(rightVelocity);
        SmartDashboard.putNumber("drivetrain/feedforward/leftVolts", leftVelocity);
        SmartDashboard.putNumber("drivetrain/feedforward/rightVolts", rightVolts);
        setVolts(leftVolts, rightVolts);
    }

    public double getMaxVelocity() {
        return ffLeft.maxAchievableVelocity(Constants.MOTOR_VOLTAGE, 0);
    }

    public double getMinVelocity() {
        return ffLeft.minAchievableVelocity(Constants.MOTOR_VOLTAGE, 0);
    }

    /**
     * Updates the PIDs on the drivetrain motors from SmartDashboard
     */
    private void updatePIDs() {
        for (TalonFX motor : driveMotors) {
            motor.config_kP(0, SmartDashboard.getNumber("drivetrain/PID/kP", 0.3));
        }
    }
}
