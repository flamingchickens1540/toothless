package org.team1540.robot2022;

import org.team1540.robot2022.utils.LinearInterpolator;

public class InterpolationTable {
    // Interpolation values for the hood down

    // Hood down
    public static final double hubFront = 2100;
    public static final double hubRear = 2100;
    public static final double tarmacFront = 2100;
    public static final double tarmacRear = 2950;

    public static final double copilotSpinupFront = 2000;
    public static final double copilotSpinupRear = 2000;
    // {98, }

    // Interpolation values for the hood up
    public static final double[][] hoodUp = {
            // {distance (inches), frontSpeed, backSpeed}
            {97,  1950, 2700},
            {110, 1950, 2850},
            {120, 1950, 2900},
            {131, 1950, 2950},
            {142, 2050, 3050},
            {151, 2050, 3050},
            {160, 2100, 3250},
            {170, 2100, 3450},
            {183, 2200, 3600},
//            {192, 3000, 3400},
//            {199, 3000, 3550},
    };

    private static InterpolationTable instance = null;
    public LinearInterpolator frontFlywheelInterpolator, rearFlywheelInterpolator;

    private InterpolationTable() {
        double[][] data = hoodUp;
        double[][] frontData = new double[data.length][2];
        double[][] rearData = new double[data.length][2];

        for (int i = 0; i < data.length; i++) {
            double[] row = data[i];
            frontData[i] = new double[]{row[0], row[1]};
            rearData[i] = new double[]{row[0], row[2]};
        }

        frontFlywheelInterpolator = new LinearInterpolator(frontData);
        rearFlywheelInterpolator = new LinearInterpolator(rearData);
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