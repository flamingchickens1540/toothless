package org.team1540.robot2022.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.Vector2d;

// import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import java.util.ArrayList;
import java.util.List;

public class Limelight {

    private static final double HORIZONTAL_FOV = Math.toRadians(59.6);
    private static final double VERTICAL_FOV = Math.toRadians(45.7);
    private static final Vector2d CAM_RESOLUTION = new Vector2d(320, 240);
    private final NetworkTable limelightTable;
    private double limelightHeight;
    private double limelightAngle;
    private double targetHeight;

    /**
     * Constructs a new limelight interface with the default hostname.
     *
     * @param name hostname of the limelight
     */
    public Limelight(String name) {
        limelightTable = NetworkTableInstance.getDefault().getTable(name);
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
     * Gets the output of the limelight targeting from the network table.
     *
     * @return a {@link Vector2D} containing the output angles of the limelight targeting in radians
     */
    public Vector2d getTargetAngles() { // TODO: This should be negated appropriately
        double x = Math.toRadians(limelightTable.getEntry("tx").getDouble(0));
        double y = Math.toRadians(limelightTable.getEntry("ty").getDouble(0));
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

    /**
     * Sets limelight to driver cam or vision mode.
     *
     * @param driverCam Whether the limelight should be in driver cam mode
    //  */
    // public void setDriverCam(boolean driverCam) {
    //     limelightTable.getEntry("camMode").setNumber(driverCam ? 1 : 0);
    //     NetworkTableInstance.getDefault().flush();
    // }

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
