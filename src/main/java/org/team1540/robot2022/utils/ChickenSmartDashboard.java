package org.team1540.robot2022.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class ChickenSmartDashboard {

    /**
     * Puts a number in the table but does not overwrite it if it already exists
     *
     * @param key          the key to be assigned to
     * @param defaultValue the value to be set if no value is found
     * @return False if the table key already exists with a different type
     * @see SmartDashboard#putNumber
     */
    public static boolean putDefaultNumber(String key, double defaultValue) {
        return SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
    }

    /**
     * Puts a boolean in the table but does not overwrite it if it already exists
     *
     * @param key          the key to be assigned to
     * @param defaultValue the value to be set if no value is found
     * @return False if the table key already exists with a different type
     * @see SmartDashboard#putBoolean
     */
    public static boolean putDefaultBoolean(String key, boolean defaultValue) {
        return SmartDashboard.putBoolean(key, SmartDashboard.getBoolean(key, defaultValue));
    }

    /**
     * Puts a String in the table but does not overwrite it if it already exists
     *
     * @param key          the key to be assigned to
     * @param defaultValue the value to be set if no value is found
     * @return False if the table key already exists with a different type
     * @see SmartDashboard#putString
     */
    public static boolean putDefaultString(String key, String defaultValue) {
        return SmartDashboard.putString(key, SmartDashboard.getString(key, defaultValue));
    }
}
