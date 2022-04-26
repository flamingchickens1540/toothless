package org.team1540.robot2022;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RamseteConfig {
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(Constants.DriveConstants.K_TRACKWIDTH_METERS);

    // private static final double kWheelCircumference = 0.4863499587;
    // private static final double kEncoderPPR = 512;
    // private static final double encoderMetersPerTick = kWheelCircumference / kEncoderPPR;

    // Motion control
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double kMaxSpeedMetersPerSecond = 3.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;

    public static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                    Constants.DriveConstants.KS_VOLTS,
                    Constants.DriveConstants.KV_VOLT_SECONDS_PER_METER,
                    Constants.DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER
            ),
            kDriveKinematics,
            10);

    // RamseteCommand Defaults
    public static final RamseteController ramseteController = new RamseteController(kRamseteB, kRamseteZeta);
    public static final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.DriveConstants.KS_VOLTS, Constants.DriveConstants.KV_VOLT_SECONDS_PER_METER, Constants.DriveConstants.KA_VOLT_SECONDS_SQUARED_PER_METER);
    public static final PIDController leftPID = new PIDController(SmartDashboard.getNumber("ramsetePID/kP", Constants.DriveConstants.KP_DRIVE_VEL), 0, 0);
    public static final PIDController rightPID = new PIDController(SmartDashboard.getNumber("ramsetePID/kP", Constants.DriveConstants.KP_DRIVE_VEL), 0, 0);
}
