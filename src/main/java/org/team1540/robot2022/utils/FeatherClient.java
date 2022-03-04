package org.team1540.robot2022.utils;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team1540.robot2022.commands.shooter.Shooter;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.Objects;
import java.util.Random;

public class FeatherClient {
    private static final Timer timer = new Timer();

    /**
     * Resets the match timer. This should be called in autonomousInit
     */
    public static void resetTimer() {
        timer.reset();
        timer.start();
    }

    /**
     * Get elapsed timer seconds
     *
     * @return timer value in seconds
     */
    public static double getTimer() {
        return timer.get();
    }

    /**
     * Get a random string
     *
     * @return random 8 character string
     */
    private static String randomString() {
        byte[] array = new byte[8];
        new Random().nextBytes(array);
        return new String(array, StandardCharsets.UTF_8);

        // return new Random().nextInt(1000).toString();
    }

    /**
     * Gets a match identifier including the event name, match and replay numbers, and alliance station.
     * If event name isn't set, this will return "practice-01234567" with a random 8 character suffix.
     *
     * @return match ID
     */
    private static String getMatchID() {
        if (Objects.equals(DriverStation.getEventName(), "")) {
            return String.format("%s-%dr%d-%s%d",
                    DriverStation.getEventName(),
                    DriverStation.getMatchNumber(),
                    DriverStation.getReplayNumber(),
                    DriverStation.getAlliance().toString(),
                    DriverStation.getLocation()
            );
        } else {
            return "practice-" + randomString();
        }
    }

    /**
     * Record a shot to a local file
     *
     * @param limelightDistance distance reported by limelight
     * @param lidarDistance     distance reported by lidar
     * @param frontRPM          front flywheel RPM
     * @param rearRPM           rear flywheel RPM
     * @param hoodUp            is the hood up?
     */
    private static void recordShotToFile(double limelightDistance, double lidarDistance,
                                         double frontRPM, double rearRPM,
                                         boolean hoodUp, Shooter.ShooterProfile profile) {
        String jsonl = String.format("{\"matchId\": \"%s\", \"timer\": %f, " +
                        "\"limelightDistance\": %f, \"lidarDistance\": %f, " +
                        "\"frontRPM\": %f, \"rearRPM\": %f, \"hoodUp\": %b, \"profile\": \"%s\"}",
                getMatchID(),
                getTimer(),
                limelightDistance, lidarDistance,
                frontRPM, rearRPM,
                hoodUp, profile.toString());

        try {
            Files.writeString(
                    Paths.get("/home/lvuser/feather.jsonl"),
                    jsonl + System.lineSeparator(),
                    StandardOpenOption.APPEND
            );
        } catch (IOException e) {
            DriverStation.reportError("[feather] Unable to append to local file: " + e, true);
        }
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
        recordShotToFile(limelightDistance, lidarDistance, frontRPM, rearRPM, hoodUp, profile);

        SmartDashboard.putString("feather/matchId", getMatchID());
        SmartDashboard.putNumber("feather/timer", getTimer());
        SmartDashboard.putNumber("feather/limelightDistance", limelightDistance);
        SmartDashboard.putNumber("feather/lidarDistance", lidarDistance);
        SmartDashboard.putNumber("feather/frontRPM", frontRPM);
        SmartDashboard.putNumber("feather/rearRPM", rearRPM);
        SmartDashboard.putBoolean("feather/hoodUp", hoodUp);
        SmartDashboard.putString("feather/profile", profile.toString());

        NetworkTableInstance.getDefault().flush();
    }
}
