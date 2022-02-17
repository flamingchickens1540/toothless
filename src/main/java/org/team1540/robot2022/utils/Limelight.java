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
    private final double limelightHeight = 0.94; // 37in to m
    private final double limelightAngle = Math.toRadians(25.5);
    private final double targetHeight = 2.64; // 8f8in to m

    /**
     * Constructs a new limelight interface with the default hostname.
     *
     * @param name hostname of the limelight
     */
    public Limelight(String name) {
        limelightTable = NetworkTableInstance.getDefault().getTable(name);
        SmartDashboard.putNumber("limelight/calculatedDistance", 0);
        // setLeds(false);
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
     * @return the distance in meters
     */
    public double getCalculatedDistance() {
        double theta = Math.toRadians(getTargetAngles().y) + limelightAngle;
        double actualHeight = targetHeight - limelightHeight;
        return actualHeight / Math.tan(theta);
    }

    /**
     * Publishes the current calculated distance (in inches) to the SmartDashboard.
     */
    public void publishCalculatedDistance() {
        SmartDashboard.putNumber("limelight/calculatedDistance", 39.37007874 * getCalculatedDistance());
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
        double y = getTargetAngles().y;
        double x = getTargetAngles().x;
        boolean verticalInBounds = y > Math.toRadians(-21);
        boolean horizontalInBounds = x > Math.toRadians(-22) && x < Math.toRadians(17);
        return (double) limelightTable.getEntry("tv").getNumber(0) > 0 && verticalInBounds && horizontalInBounds;
    }

    public boolean getLeds() {
        return limelightTable.getEntry("ledMode").getDouble(1) == 0;
    }

    /**
     * Sets limelight's green LEDs on or off.
     *
     * @param isOn the new state of the LEDs
     */
    public void setLeds(boolean isOn) {
        if (getLeds() != isOn) {
            limelightTable.getEntry("ledMode").setNumber(isOn ? 0 : 1);
            NetworkTableInstance.getDefault().flush();
        }
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
