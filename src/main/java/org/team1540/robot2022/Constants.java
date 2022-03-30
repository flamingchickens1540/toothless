// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team1540.robot2022;

import org.team1540.robot2022.utils.CurrentLimitConfig;

public final class Constants {
    public static final int PNEUMATIC_HUB = 20;

    public static final double MOTOR_VOLTAGE = 12;

    public static final class DriveConstants {
        public static final double ENCODER_TICKS_PER_METER = 49866;

        public static final class Motors {
            public static final int LEFT_FRONT = 3;
            public static final int LEFT_REAR = 4;
            public static final int RIGHT_FRONT = 1;
            public static final int RIGHT_REAR = 2;
        }

        // Calculated in frc-characterization
        public static final double KS_VOLTS = 0.650;
        public static final double KV_VOLT_SECONDS_PER_METER = 2.81;
        public static final double KA_VOLT_SECONDS_SQUARED_PER_METER = 0.224;

        // Ramsete PID controllers
        public static final double KP_DRIVE_VEL = 3.2925;

        public static final double K_TRACKWIDTH_METERS = 0.6604;
    }

    public static final class HoodConstants {
        public static final int SOLENOID_CHANNEL = 2;
    }

    public static final class IntakeConstants {
        public static final int SOLENOID = 3;
        public static final int FALCON = 5;
        public static final CurrentLimitConfig CURRENT_LIMIT_CONFIG = new CurrentLimitConfig(20, 25, 1, 10, 15, 0.5);

        public static final double SPEED = 0.75;
    }

    public static final class ShooterConstants {
        public static final int FRONT = 9;
        public static final int REAR = 8;

        public static final CurrentLimitConfig CURRENT_LIMIT_CONFIG = new CurrentLimitConfig(20, 25, 1, 10, 15, 0.5);
    }

    public static final class IndexerConstants {
        public static final class IndexerMotors {
            public static final int BOTTOM_MOTOR = 6;
            public static final int TOP_MOTOR = 7;

            public static final CurrentLimitConfig CURRENT_LIMIT_CONFIG = new CurrentLimitConfig(20, 25, 1, 10, 15, 0.5);
        }

        public static final class BeamBreaks {
            public static final int TOP_INDEXER_SENSOR = 8;
            public static final int BOTTOM_INDEXER_SENSOR = 9;
        }

        public static final double TOP_PERCENT = 0.4;
        public static final double BOTTOM_PERCENT = 0.5;
    }

    public static final class ClimberConstants {
        public static final class Motors {
            public static final int RIGHT = 10;
            public static final int LEFT = 11;
        }

        public static final class Solenoids {
            public static final int SOLENOID_A = 0;
            public static final int SOLENOID_B = 1;
        }

        // Zero sequence
        public static final double ZERO_DOWN_SPEED = 0.8;
        public static final double ZERO_SPIKE_CURRENT = 20;
    }

}
