package org.team1540.robot2022;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RamseteConfig {
    // Calculated in frc-characterization
    public static final double ksVolts = 0.650;
    public static final double kvVoltSecondsPerMeter = 2.81;
    public static final double kaVoltSecondsSquaredPerMeter = 0.224;

    // Ramsete PID controllers
    public static final double kPDriveVel = 3.2925;

    private static final double kTrackwidthMeters = 0.6604;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            kTrackwidthMeters);

    // private static final double kWheelCircumference = 0.4863499587;
    // private static final double kEncoderPPR = 512;
    // private static final double encoderMetersPerTick = kWheelCircumference /
    // kEncoderPPR;

    // Motion control
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double kMaxSpeedMetersPerSecond = 1.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    public static final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter),
            kDriveKinematics,
            10);

    // RamseteCommand Defaults
    public static final RamseteController ramseteController = new RamseteController(kRamseteB, kRamseteZeta);
    public static final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter,
            kaVoltSecondsSquaredPerMeter);
    public static final PIDController leftPID = new PIDController(SmartDashboard.getNumber("ramsetePID/kP", kPDriveVel), 0, 0);
    public static final PIDController rightPID = new PIDController(SmartDashboard.getNumber("ramsetePID/kP", kPDriveVel), 0,
            0);


}