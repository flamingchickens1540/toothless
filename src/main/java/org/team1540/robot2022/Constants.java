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
    public static final class DriveConstants {
        public static final double encoderTicksPerMeter = 49866;

        public static final class Motors {
            public static final int leftFront = 1;
            public static final int leftRear = 2;
            public static final int rightFront = 3;
            public static final int rightRear = 4;
        }

    }

    public static final class HoodConstants {
        public static final int solenoidChannel = 0;
    }

    public static final class IntakeConstants {
        public static final int leftSolenoidChannel = 0;
        public static final int rightSolenoidChannel = 0;
        public static final int falcon = 0;
        public static final CurrentLimitConfig currentLimitConfig = new CurrentLimitConfig(20, 25, 1, 10, 15, 0.5);
    }

    public static final class ShooterConstants {
        public static final int front = 0;
        public static final int rear = 0;

        public static final CurrentLimitConfig currentLimitConfig = new CurrentLimitConfig(20, 25, 1, 10, 15, 0.5);
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
