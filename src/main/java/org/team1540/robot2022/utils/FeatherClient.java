package org.team1540.robot2022.utils;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.team1540.robot2022.commands.shooter.Shooter;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.Objects;
import java.util.UUID;

public class FeatherClient {
    private static final Timer timer = new Timer();
    private static final Mat mat = new Mat();

    private static ShootingParameters lastShot;

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
     * Gets a match identifier including the event name, match and replay numbers, and alliance station.
     *
     * @return match ID
     */
    private static String getMatchID() {
        DriverStation.waitForData();
        String eventName = DriverStation.getEventName();
        return String.format("%s-%dr%d-%s%d", !Objects.equals(eventName, "") ? eventName : "unknown", DriverStation.getMatchNumber(), DriverStation.getReplayNumber(), DriverStation.getAlliance().toString(), DriverStation.getLocation());
    }

    /**
     * Record a shot and result to a local file
     *
     * @param shot       shooting parameters
     * @param firstBall  what happened to the first ball?
     * @param secondBall what happened to the second ball?
     */
    private static void commitShot(ShootingParameters shot, ShotResult firstBall, ShotResult secondBall) {
        String jsonString = String.format("{\"matchId\": \"%s\", \"timer\": %f, " + "\"limelightDistance\": %f, \"lidarDistance\": %f, " + "\"frontRPM\": %f, \"rearRPM\": %f, \"hoodUp\": %b, \"profile\": \"%s\", " + "\"firstResult\": %s, \"secondResult\": %s}", shot.matchId, lastShot.matchSeconds, shot.limelightDistance, shot.lidarDistance, shot.frontRPM, shot.rearRPM, shot.hoodUp, shot.profile + "", firstBall + "", secondBall + "");

        // Write shot log to file
        try {
            Files.writeString(Paths.get(shot.directoryPrefix + "/shot.json"), jsonString + System.lineSeparator(), StandardOpenOption.CREATE);
        } catch (IOException e) {
            DriverStation.reportError("[feather] Unable to create shot log file: " + e, true);
        }
    }

    /**
     * Record a shot
     *
     * @param parameters shooting parameters
     */
    private static void recordShot(ShootingParameters parameters) {
        if (lastShot != null) { // If the last shot hasn't been committed yet, commit it with unknown results
            commitShot(parameters, ShotResult.UNKNOWN, ShotResult.UNKNOWN);
        }
        lastShot = parameters;
    }

    /**
     * Set the last shot's ball state
     *
     * @param result what happened to the ball?
     */
    private static void confirmShot(ShotResult result) {
        if (lastShot != null) { // If the last shot's shooting parameters have been recorded...
            if (lastShot.firstBall == null) { // If the first ball's result hasn't been recorded yet, record the result
                lastShot.firstBall = result;
            } else if (lastShot.secondBall == null) { // If the second ball's result hasn't been recorded yet, record the result
                lastShot.secondBall = result;
            }
            // If both balls have been recorded already, ignore this shot result
        }
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
     * @return InstantCommand to record a shot
     */
    public static Command commandRecordShot(double limelightDistance, double lidarDistance, double frontRPM, double rearRPM, boolean hoodUp, Shooter.ShooterProfile profile) {
        return new InstantCommand(() -> recordShot(new ShootingParameters(limelightDistance, lidarDistance, frontRPM, rearRPM, hoodUp, profile)));
    }

    /**
     * What happened to the ball after it left the robot?
     */
    public enum ShotResult {
        /**
         * Shot went in
         */
        OK,

        /**
         * Shot went too far
         */
        OVER,

        /**
         * Shot didn't go far enough
         */
        UNDER,

        /**
         * Shot bounced out
         */
        BOUNCED,

        /**
         * We don't know what happened
         */
        UNKNOWN
    }

    /**
     * Shooting parameters
     */
    private static class ShootingParameters {
        public double limelightDistance;
        public double lidarDistance;
        public double frontRPM;
        public double rearRPM;
        public boolean hoodUp;
        public Shooter.ShooterProfile profile;

        public String matchId;
        public double matchSeconds;
        public ShotResult firstBall;
        public ShotResult secondBall;
        public String directoryPrefix;

        public ShootingParameters(double limelightDistance, double lidarDistance, double frontRPM, double rearRPM, boolean hoodUp, Shooter.ShooterProfile profile) {
            this.limelightDistance = limelightDistance;
            this.lidarDistance = lidarDistance;
            this.frontRPM = frontRPM;
            this.rearRPM = rearRPM;
            this.hoodUp = hoodUp;
            this.profile = profile;

            this.matchId = getMatchID();
            this.matchSeconds = getTimer();

            // Generate a directory name with match ID and a random UUID to handle tethered testing and FMS problems
            this.directoryPrefix = "/home/lvuser/feather/" + this.matchId + "_" + UUID.randomUUID();

            // Create match directory
            File directory = new File(this.directoryPrefix);
            if (!directory.exists()) directory.mkdirs();

            // Write limelight image
            CameraServer.getVideo("limelight").grabFrame(mat);
            Imgcodecs.imwrite(this.directoryPrefix + "/limelight-" + this.matchId + ".png", mat);
        }
    }

    /**
     * Configures button bindings on an xbox controller
     *
     * @param controller xbox controller
     */
    public static void configureController(XboxController controller) {
        // coop:button(Y,Over,pilot)
        new JoystickButton(controller, XboxController.Button.kY.value)
                .whenPressed(new InstantCommand(() -> confirmShot(ShotResult.OVER)));
        // coop:button(X,OK,pilot)
        new JoystickButton(controller, XboxController.Button.kX.value)
                .whenPressed(new InstantCommand(() -> confirmShot(ShotResult.OK)));
        // coop:button(B,Bounced,pilot)
        new JoystickButton(controller, XboxController.Button.kB.value)
                .whenPressed(new InstantCommand(() -> confirmShot(ShotResult.BOUNCED)));
        // coop:button(A,Under,pilot)
        new JoystickButton(controller, XboxController.Button.kA.value)
                .whenPressed(new InstantCommand(() -> confirmShot(ShotResult.UNDER)));
    }
}
