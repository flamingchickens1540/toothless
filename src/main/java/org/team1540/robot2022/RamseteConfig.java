package org.team1540.robot2022;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;


public class RamseteConfig {
    //    Calculated in frc-characterization
    public static final double ksVolts = 0.127;
    public static final double kvVoltSecondsPerMeter = 2.6;
    public static final double kaVoltSecondsSquaredPerMeter = 0.292;

    //    Ramsete PID controllers
    public static final double kPDriveVel = 0.7;

    private static final double kTrackwidthMeters = 0.67978793613;
    public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

//    private static final double kWheelCircumference = 0.4863499587;
//    private static final double kEncoderPPR = 512;
//    private static final double encoderMetersPerTick = kWheelCircumference / kEncoderPPR;

    // Motion control
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double kMaxSpeedMetersPerSecond = 0.2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.1;

    public static final DifferentialDriveVoltageConstraint autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter),
                    kDriveKinematics,
                    10);

}