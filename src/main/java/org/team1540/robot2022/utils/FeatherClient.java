package org.team1540.robot2022.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FeatherClient {
    /**
     * Record a shot for the feather backend
     *
     * @param distanceFromTarget distance from target from limelight in inches
     * @param frontRPM           front flywheel RPM
     * @param rearRPM            rear flywheel RPM
     * @param hoodUp             is the hood up?
     */
    public static void recordShot(double distanceFromTarget, double frontRPM, double rearRPM, boolean hoodUp) {
        SmartDashboard.putBoolean("feather/shotTaken", false);

        SmartDashboard.putNumber("feather/matchTime", DriverStation.getMatchTime());
        SmartDashboard.putString("feather/matchId", String.format("%s-%dr%d-%s",
                DriverStation.getEventName(),
                DriverStation.getMatchNumber(),
                DriverStation.getReplayNumber(),
                DriverStation.getMatchType()
        ));

        SmartDashboard.putNumber("feather/distanceFromTarget", distanceFromTarget);
        SmartDashboard.putNumber("feather/frontRPM", frontRPM);
        SmartDashboard.putNumber("feather/rearRPM", rearRPM);
        SmartDashboard.putBoolean("feather/hoodUp", hoodUp);

        SmartDashboard.putBoolean("feather/shotTaken", true);
    }
}