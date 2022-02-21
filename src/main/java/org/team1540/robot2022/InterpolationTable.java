package org.team1540.robot2022;

import org.team1540.robot2022.utils.LinearInterpolator;

public class InterpolationTable {
    private static InterpolationTable instance = null;
    public static final double[][] data = {
            // {distance (inches), frontSpeed, backSpeed}
            {0, -2000, -2000},
            {150, -2000, -3250},
    };

    public LinearInterpolator frontFlywheelInterpolator, rearFlywheelInterpolator;

    private InterpolationTable() {
        double[][] frontData = new double[data.length][2];
        double[][] rearData = new double[data.length][2];

        for (int i = 0; i < data.length; i++) {
            double[] row = data[i];
            frontData[i] = new double[] {row[0], row[1]};
            rearData[i] = new double[] {row[0], row[2]};
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
