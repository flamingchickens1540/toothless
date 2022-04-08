package org.team1540.robot2022.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.team1540.robot2022.commands.util.UpdateMatchInfo;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.UUID;

public class FeatherClient {
    private static final Timer timer = new Timer();
    //    private static final Mat mat = new Mat();
    private static String matchDirectoryPrefix;

    /**
     * Attempt to update matchId when the robot receives the FMS matchInfo packet
     *
     * @return true if the update was successful
     */
    public static boolean updateMatchId() {
        if (DriverStation.getMatchType() != DriverStation.MatchType.None) {
            String matchId = String.format("%s-%dr%d-%s%d",
                    DriverStation.getEventName(),
                    DriverStation.getMatchNumber(),
                    DriverStation.getReplayNumber(),
                    DriverStation.getAlliance() + "",
                    DriverStation.getLocation());

            // Write shot log to file
            try {
                Files.writeString(Paths.get(matchDirectoryPrefix + "/matchId"), matchId + System.lineSeparator(), StandardOpenOption.CREATE);
                return true;
            } catch (IOException e) {
                DriverStation.reportError("[feather] Unable to create matchId file: " + e, true);
            }
        }
        return false;
    }

    /**
     * Initialize a feather match. This should be called in autonomousInit.
     * - Resets the match timer
     * - Creates the directory structure
     * - Schedules the match info update command
     */
    public static void initialize() {
        timer.reset();
        timer.start();

        matchDirectoryPrefix = "/home/lvuser/feather/matches/" + UUID.randomUUID();

        // Create match directory
        File directory = new File(matchDirectoryPrefix);
        if (!directory.exists()) directory.mkdirs();

        System.out.println("[feather] Initialized match directory at " + matchDirectoryPrefix);

        // Create shot log file
        try {
            Files.writeString(Paths.get(matchDirectoryPrefix + "/shots.jsonl"), "", StandardOpenOption.CREATE);
        } catch (IOException e) {
            DriverStation.reportError("[feather] Unable to create shot log file: " + e, true);
        }

//        new UpdateMatchInfo().schedule();
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
     * Return an instant command that records a shot for the feather backend
     *
     * @param limelightDistance distance from limelight
     * @param lidarDistance     distance from LIDAR
     * @param frontRPM          front RPM setpoint
     * @param rearRPM           rear RPM setpoint
     * @param hoodUp            is the hood up?
     * @param profile           shooter profile
     */
    public static void recordShot(double limelightDistance, double lidarDistance,
                                  double frontRPM, double rearRPM, boolean hoodUp,
                                  String profile) {
        String jsonString = String.format("{\"id\": \"%s\", \"timer\": %f, " +
                        "\"limelightDistance\": %f, \"lidarDistance\": %f, " +
                        "\"frontRPM\": %f, \"rearRPM\": %f, \"hoodUp\": %b, \"profile\": \"%s\"}",
                UUID.randomUUID() + "", getTimer(),
                limelightDistance, lidarDistance,
                frontRPM, rearRPM, hoodUp, profile
        );

        // Write shot log to file
//        lastShot = null;
        try {
            Files.writeString(Paths.get(matchDirectoryPrefix + "/shots.jsonl"), jsonString + System.lineSeparator(), StandardOpenOption.APPEND);
        } catch (IOException e) {
            DriverStation.reportError("[feather] Unable to append to shot log file: " + e, true);
        }
    }
}
