package org.team1540.robot2022.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class ChickenSmartDashboard {

    /**
     * Puts a number in the table but does not overwrite it if it already exists
     *
     * @param key          the key to be assigned to
     * @param defaultValue the value to be set if no value is found
     * @see SmartDashboard#putNumber
     */
    public static void putDefaultNumber(String key, double defaultValue) {
        SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
        SmartDashboard.setPersistent(key);
    }

    /**
     * Puts a boolean in the table but does not overwrite it if it already exists
     *
     * @param key          the key to be assigned to
     * @param defaultValue the value to be set if no value is found
     * @see SmartDashboard#putBoolean
     */
    public static void putDefaultBoolean(String key, boolean defaultValue) {
        SmartDashboard.putBoolean(key, SmartDashboard.getBoolean(key, defaultValue));
        SmartDashboard.setPersistent(key);
    }

    /**
     * Puts a String in the table but does not overwrite it if it already exists
     *
     * @param key          the key to be assigned to
     * @param defaultValue the value to be set if no value is found
     * @see SmartDashboard#putString
     */
    public static void putDefaultString(String key, String defaultValue) {
        SmartDashboard.putString(key, SmartDashboard.getString(key, defaultValue));
        SmartDashboard.setPersistent(key);
    }

    /**
     * Puts a number in the table if the robot is not connected to an FMS.
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @see SmartDashboard#putNumber
     */
    public static void putDebugNumber(String key, double value) {
        if (!DriverStation.isFMSAttached()) {
            SmartDashboard.putNumber(key, value);
        }
    }

    /**
     * Puts a boolean in the table if the robot is not connected to an FMS.
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @see SmartDashboard#putBoolean
     */
    public static void putDebugBoolean(String key, boolean value) {
        if (!DriverStation.isFMSAttached()) {
            SmartDashboard.putBoolean(key, value);
        }
    }

    /**
     * Puts a string in the table if the robot is not connected to an FMS.
     *
     * @param key   the key to be assigned to
     * @param value the value that will be assigned
     * @see SmartDashboard#putString
     */
    public static void putDebugString(String key, String value) {
        if (!DriverStation.isFMSAttached()) {
            SmartDashboard.putString(key, value);
        }
    }
}
