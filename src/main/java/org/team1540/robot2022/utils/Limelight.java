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
    public final double limelightHeight = 0.71; // 28in to m
    public final double limelightAngle = Math.toRadians(40);
    public final double targetHeight = 2.64; // 8'8" to m

    /**
     * Constructs a new limelight interface with the default hostname.
     *
     * @param name hostname of the limelight
     */
    public Limelight(String name) {
        limelightTable = NetworkTableInstance.getDefault().getTable(name);
        SmartDashboard.putNumber("limelight/custom/calculatedDistance", 0);
        SmartDashboard.putNumber("limelight/custom/targetAlignedRange", 5);
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

    public void updateSmartDashboardValues() {
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
        double angle = Math.abs(getTargetAngles().x);
        return angle > 0.001;
    }

    /**
     * Tells whether the limelight target is aligned to within 1 degree.
     *
     * @return the state of target alignment
     */
    public boolean isTargetAligned() {
        double distance = Math.abs(getTargetAngles().x);
        return distance > 0 && distance < SmartDashboard.getNumber("limelight/custom/targetAlignedRange", 5);
    }

    public double getLeds() {
        return limelightTable.getEntry("ledMode").getDouble(1);
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

    public boolean getLedsOn() {
        return limelightTable.getEntry("ledMode").getDouble(1) == LEDMode.ON;
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

    public static final class LEDMode {
        public static final int PIPELINE = 0;
        public static final int OFF = 1;
        public static final int BLINK = 2;
        public static final int ON = 3;
    }
}
