package org.team1540.robot2022.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class ChickenSmartDashboard {

    public static boolean putDefaultNumber(String key, double defaultValue) {
        return SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
    }

    public static boolean putDefaultBoolean(String key, boolean defaultValue) {
        return SmartDashboard.putBoolean(key, SmartDashboard.getBoolean(key, defaultValue));
    }

    public static boolean putDefaultString(String key, String defaultValue) {
        return SmartDashboard.putString(key, SmartDashboard.getString(key, defaultValue));
    }
}
