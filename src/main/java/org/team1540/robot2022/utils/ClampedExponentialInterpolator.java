package org.team1540.robot2022.utils;

/**
 * ClampedExponentialInterpolator returns an exponential value given an input, leading coefficient, base, and upper and lower inclusive bounds.
 */
public class ClampedExponentialInterpolator implements Interpolator {
    private final double a, b, lower, upper;

    public ClampedExponentialInterpolator(double a, double b, double lower, double upper) {
        this.a = a;
        this.b = b;
        this.lower = lower;
        this.upper = upper;
    }

    @Override
    public double getInterpolatedValue(double x) {
        double output = a * Math.pow(b, x);
        if (output <= lower) {
            return lower;
        } else if (output >= upper) {
            return upper;
        } else {
            return output;
        }
    }
}
