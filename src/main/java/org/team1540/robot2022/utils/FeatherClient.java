package org.team1540.robot2022.utils;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team1540.robot2022.commands.shooter.Shooter;

import java.nio.charset.StandardCharsets;
import java.util.Objects;
import java.util.Random;

public class FeatherClient {
    private static String randomString() {
        byte[] array = new byte[8];
        new Random().nextBytes(array);
        return new String(array, StandardCharsets.UTF_8);
    }

    /**
     * Record a shot for the feather backend
     *
     * @param limelightDistance distance reported by limelight
     * @param lidarDistance     distance reported by lidar
     * @param frontRPM          front flywheel RPM
     * @param rearRPM           rear flywheel RPM
     * @param hoodUp            is the hood up?
     */
    public static void recordShot(double limelightDistance, double lidarDistance, double frontRPM, double rearRPM, boolean hoodUp, Shooter.ShooterProfile profile) {
        SmartDashboard.putNumber("feather/matchTime", DriverStation.getMatchTime());

        if (Objects.equals(DriverStation.getEventName(), "")) {
            SmartDashboard.putString("feather/matchId", String.format("%s-%dr%d-%s",
                    DriverStation.getEventName(),
                    DriverStation.getMatchNumber(),
                    DriverStation.getReplayNumber(),
                    DriverStation.getMatchType()
            ));
        } else {
            SmartDashboard.putString("feather/matchId", "practice-" + randomString());
        }

        SmartDashboard.putNumber("feather/limelightDistance", limelightDistance);
        SmartDashboard.putNumber("feather/lidarDistance", lidarDistance);
        SmartDashboard.putNumber("feather/frontRPM", frontRPM);
        SmartDashboard.putNumber("feather/rearRPM", rearRPM);
        SmartDashboard.putBoolean("feather/hoodUp", hoodUp);
        SmartDashboard.putString("feather/profile", profile.toString());

        NetworkTableInstance.getDefault().flush();
    }
}
