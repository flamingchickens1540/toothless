package org.team1540.robot2022;

import org.team1540.robot2022.utils.ClampedExponentialInterpolator;
import org.team1540.robot2022.utils.Interpolator;
import org.team1540.robot2022.utils.LinearInterpolator;

public class InterpolationTable {
    // Hood down
    public static final double hubFront = 1800;
    public static final double hubRear = 1900;

    public static final double lowGoalFront = 1000;
    public static final double lowGoalRear = 1000;

    public static final double tarmacFront = 1000;
    public static final double tarmacRear = 1000;

    public static final double shooterSpinupFront = 2000;
    public static final double shooterSpinupRear = 2000;

    // Interpolation values for the hood up
    public static final double[][] hoodUp = {
            // {distance (inches), frontSpeed, backSpeed}
            {60, 2000, 1900},
            {78, 2000, 2000},
            {92, 2100, 2100},
            {104, 2200, 2200},
            {128, 2000, 2800},
            {139, 2000, 3100},
            {151, 2000, 3300},
            {162, 2000, 4000},
    };

    private static InterpolationTable instance = null;
    public Interpolator frontFlywheelInterpolator, rearFlywheelInterpolator;

    private InterpolationTable() {
        double[][] data = hoodUp;
        double[][] frontData = new double[data.length][2];
        double[][] rearData = new double[data.length][2];

        for (int i = 0; i < data.length; i++) {
            double[] row = data[i];
            frontData[i] = new double[]{row[0], row[1]};
            rearData[i] = new double[]{row[0], row[2]};
        }

//        frontFlywheelInterpolator = new LinearInterpolator(frontData);
        // Exponential values from https://www.desmos.com/calculator/u2hiylp0ry
        frontFlywheelInterpolator = new LinearInterpolator(frontData);
        rearFlywheelInterpolator = new ClampedExponentialInterpolator(1041.79, 1.00796, 2000, 4000);
    }

    public static InterpolationTable getInstance() {
        if (instance == null) {
            instance = new InterpolationTable();
        }
        return instance;
    }
}

/*
// Front
131, -1950
142, -2050
151, -2050
160, -2100
170, -2100
183, -2200
192, -3000
199, -3000
247, -3700

// Back
131, -2950
142, -3050
151, -3050
160, -3250
170, -3450
183, -3600
192, -3400
199, -3550
247, -4800
 */