package org.team1540.robot2022.utils;

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
}
