package org.team1540.robot2022.utils;

public class MathUtils {
    /**
     * Returns 0 if the input is within the deadzone, else the value
     *
     * @param value The joystick input
     * @return The value after deadzone is checked
     */
    public static double deadzone(double value, double deadzone) {
        if (java.lang.Math.abs(value) <= deadzone)
            return 0;
        else
            return value;
    }
}
