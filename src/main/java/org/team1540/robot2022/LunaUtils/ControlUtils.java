package org.team1540.robot2022.LunaUtils;

public class ControlUtils {

    public static double velocityPosNegConstrain(double velocity, double maxVelocity, double minVelocity) {
        if (Math.abs(velocity) > maxVelocity) {
            velocity = Math.copySign(maxVelocity, velocity);
        } else if (Math.abs(velocity) < minVelocity) {
            velocity = Math.copySign(minVelocity, velocity);
        }
        return velocity;
    }

    public static double simpleDeadzone(double input, double deadzone) {
        if (Math.abs(input) < Math.abs(deadzone)) {
            return 0;
        }
        return input;
    }

    public static double allVelocityConstraints(double output, double max, double min, double deadzone) {
        return simpleDeadzone(velocityPosNegConstrain(output, max, min), deadzone);
    }

    /**
     * https://www.desmos.com/calculator/ybuyhcfzgm
     *
     * @param input x
     * @param fast  f
     * @param slow  s
     * @param fastX b, fastX > slowX
     * @param slowX a
     * @return y
     */
    public static double linearDeadzoneRamp(double input, boolean allowNegativeValues, double fast, double slow, double fastX, double slowX) {
        double slope = (fast - slow) / (fastX - slowX);
        double absInput = Math.abs(input);
        double rawResult = slope * (absInput - slowX) + slow;
        if (absInput < slowX) {
            rawResult = slow;
        } else if (absInput > fastX) {
            rawResult = fast;
        }
        rawResult = Math.copySign(rawResult, input);
        if (allowNegativeValues) {
            return rawResult;
        }
        if (rawResult < 0) {
            rawResult = slow;
        }
        return rawResult;
    }
}

