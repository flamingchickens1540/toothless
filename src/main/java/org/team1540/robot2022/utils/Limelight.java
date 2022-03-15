package org.team1540.robot2022.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;

public class Limelight {
    private static final double HORIZONTAL_FOV = Math.toRadians(59.6);
    private static final double VERTICAL_FOV = Math.toRadians(49.7);
    private static final Vector2d CAM_RESOLUTION = new Vector2d(320, 240);
    private final NetworkTable limelightTable;
    private final double limelightHeight = 0.71; // 28in to m
    private final double limelightAngle = Math.toRadians(39);
    private final double targetHeight = 2.64; // 8'8" to m

    /**
     * Constructs a new limelight interface with the default hostname.
     *
     * @param name hostname of the limelight
     */
    public Limelight(String name) {
        limelightTable = NetworkTableInstance.getDefault().getTable(name);
        SmartDashboard.putNumber("limelight/custom/calculatedDistance", 0);
        setLeds(true);
    }

    public NetworkTable getNetworkTable() {
        return limelightTable;
    }

    public double getHorizontalFov() {
        return HORIZONTAL_FOV;
    }

    public double getVerticalFov() {
        return VERTICAL_FOV;
    }

    public Vector2d getResolution() {
        return CAM_RESOLUTION;
    }

    /**
     * Gets the current calculated distance of the limelight to the base of the hub.
     *
     * @return the distance in inches
     */
    public double getCalculatedDistance() {
        double theta = Math.toRadians(getTargetAngles().y) + limelightAngle;
        double actualHeight = targetHeight - limelightHeight;
        return 39.37007874 * actualHeight / Math.tan(theta);
    }

    public void updateSmartDashboardValues() {
        SmartDashboard.putNumber("limelight/custom/calculatedDistance", getCalculatedDistance());
        SmartDashboard.putBoolean("limelight/custom/targetFound", isTargetFound());
    }

    /**
     * Gets the output of the limelight targeting from the network table.
     *
     * @return a {@link Vector2d} containing the output angles of the limelight targeting in degrees
     */
    public Vector2d getTargetAngles() {
        double x = limelightTable.getEntry("tx").getDouble(0);
        double y = limelightTable.getEntry("ty").getDouble(0);
        return new Vector2d(x, y);
    }

    /**
     * Queries whether the limelight target has been found.
     *
     * @return the state of the target
     */
    public boolean isTargetFound() {
        return getTargetAngles().x != 0;
    }

    public double getLeds() {
        return limelightTable.getEntry("ledMode").getDouble(1);
    }

    public boolean getLedsOn() {
        return limelightTable.getEntry("ledMode").getDouble(1) == LEDMode.ON;
    }

    /**
     * Sets limelight's green LEDs on or off.
     *
     * @param isOn If the LEDs should be on, otherwise off
     */
    public void setLeds(boolean isOn) {
        setLeds(isOn ? LEDMode.ON : LEDMode.OFF);
    }

    /**
     * Sets limelight's green LEDs on or off.
     *
     * @param mode the new state of the LEDs
     */
    public void setLeds(int mode) {
        if (getLeds() != mode) {
            limelightTable.getEntry("ledMode").setNumber(mode);
            NetworkTableInstance.getDefault().flush();
        }
    }

    public static final class LEDMode {
        public static final int PIPELINE = 0;
        public static final int OFF = 1;
        public static final int BLINK = 2;
        public static final int ON = 3;
    }

    public long getPipeline() {
        return Math.round((double) limelightTable.getEntry("getpipe").getNumber(-1));
    }

    public void setPipeline(double id) {
        if (getPipeline() != id) {
            limelightTable.getEntry("pipeline").setNumber(id);
            NetworkTableInstance.getDefault().flush();
        }
    }

    public List<Vector2d> getCorners() {
        Double[] xCorners = limelightTable.getEntry("tcornx").getDoubleArray(new Double[]{});
        Double[] yCorners = limelightTable.getEntry("tcorny").getDoubleArray(new Double[]{});
        List<Vector2d> cornerList = new ArrayList<>();
        for (int i = 0; i < xCorners.length; i++) {
            cornerList.add(new Vector2d(xCorners[i], yCorners[i]));
        }
        return cornerList;
    }
}
