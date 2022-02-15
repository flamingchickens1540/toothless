// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team1540.robot2022;

import org.team1540.robot2022.utils.CurrentLimitConfig;
import org.team1540.robot2022.utils.RevBlinken.ColorPattern;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
    public static final int ph = 20;

    public static final class DriveConstants {
        public static final double encoderTicksPerMeter = 49866;

        public static final class Motors {
            public static final int leftFront = 3;
            public static final int leftRear = 4;
            public static final int rightFront = 1;
            public static final int rightRear = 2;
        }
    }

    public static final class HoodConstants {
        public static final int solenoidChannel = 2;
    }

    public static final class IntakeConstants {
        public static final int solenoid = 3;
        public static final int falcon = 5;
        public static final CurrentLimitConfig currentLimitConfig = new CurrentLimitConfig(20, 25, 1, 10, 15, 0.5);
    }

    public static final class ShooterConstants {
        public static final int front = 9;
        public static final int rear = 8;

        public static final CurrentLimitConfig currentLimitConfig = new CurrentLimitConfig(20, 25, 1, 10, 15, 0.5);
    }

    public static final class IndexerConstants {
        public static final class IndexerMotors {
            public static final int bottomMotor = 6;
            public static final int topMotor = 7;

            public static final CurrentLimitConfig currentLimitConfig = new CurrentLimitConfig(20, 25, 1, 10, 15, 0.5);
        }
        public static final class BeamBreaks {
            public static final int topIndexerSensor = 8;
            public static final int bottomIndexerSensor = 9;
        }

        public static final double topPercent = 0.5;
        public static final double bottomPercent = 0.8;
    }

    public static final class LightConstants {
        public static final ColorPattern disable = ColorPattern.FIRE_LARGE;

        public static final ColorPattern redAuto = ColorPattern.BPM_LAVA;
        public static final ColorPattern blueAuto = ColorPattern.BPM_OCEAN;

        public static final ColorPattern redTeleop = ColorPattern.RAINBOW_LAVA;
        public static final ColorPattern blueTeleop = ColorPattern.RAINBOW_OCEAN;

        public static final ColorPattern redEndgame = ColorPattern.RAINBOW_PARTY;
        public static final ColorPattern blueEndgame = ColorPattern.RAINBOW_PARTY;
    }
}
