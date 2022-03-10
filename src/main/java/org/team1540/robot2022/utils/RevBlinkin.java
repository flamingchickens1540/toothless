package org.team1540.robot2022.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Wrapper for Rev Robotics Blinkin LED
 * Driver.
 */
public class RevBlinkin extends Spark {

    private final boolean isTop;

    /**
     * Construct an instance of a RevBlinkin
     *
     * @param channel The PWM channel that the Blinkin is attached to. 0-9 are
     *                on-board, 10-19 are on
     *                the MXP port
     * @throws IndexOutOfBoundsException If the specified PWM channel does not
     *                                   exist.
     */
    public RevBlinkin(int channel, boolean isTop) {
        super(channel);

        this.isTop = isTop;
    }

    /**
     * Sets the light pattern.
     *
     * @param pattern The pattern to use
     * @throws NullPointerException If pattern is null.
     */
    public void set(ColorPattern pattern) {
        super.set(pattern.setpoint);
    }

    /**
     * Sets the Blinkin output manually.
     *
     * @param manualSetpoint The manual setpoint to be sent over PWM.
     */
    public void set(double manualSetpoint) {
        super.set(manualSetpoint);
    }

    /**
     * Applies the pattern set for a game stage
     *
     * @param stage What part of the game to set the pattern for
     */
    public void applyPattern(GameStage stage) {
        ColorPattern pattern = this.isTop ? stage.getTop() : stage.getBottom();
        SmartDashboard.putString("lights/pattern" + (isTop ? "Top" : "Bottom"), pattern + "");
        this.set(pattern);
    }

    /**
     * Enum for possible game stages
     */
    public enum GameStage {
        AUTONOMOUS(ColorPattern.FIRE_MEDIUM, PatternTheme.RAINBOW),
        TELEOP(ColorPattern.FIRE_MEDIUM, PatternTheme.RAINBOW),
        ENDGAME(ColorPattern.RAINBOW_PARTY, ColorPattern.RAINBOW_PARTY),
        DISABLE(ColorPattern.TWINKLES_RAINBOW, ColorPattern.TWINKLES_RAINBOW);

        private final PatternTheme top;
        private final PatternTheme bottom;

        GameStage(PatternTheme top, PatternTheme bottom) {
            this.top = top;
            this.bottom = bottom;
        }

        GameStage(ColorPattern top, PatternTheme bottom) {
            this.top = new PatternTheme(top);
            this.bottom = bottom;
        }

        GameStage(ColorPattern top, ColorPattern bottom) {
            this.top = new PatternTheme(top);
            this.bottom = new PatternTheme(bottom);
        }

        /**
         * Gets the pattern for the top set of lights
         *
         * @return the pattern
         */
        public ColorPattern getTop() {
            return this.top.get();
        }

        /**
         * Gets the pattern for the bottom set of lights
         *
         * @return the pattern
         */
        public ColorPattern getBottom() {
            return this.bottom.get();
        }
    }

    /**
     * Enum for possible color patterns according to the
     * <a href="http://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf">Blinkin
     * user manual.</a>
     */
    private enum ColorPattern {
        RAINBOW(-0.99),
        RAINBOW_PARTY(-0.97),
        RAINBOW_OCEAN(-0.95),
        RAINBOW_LAVA(-0.93),
        RAINBOW_FOREST(-0.91),
        RAINBOW_GLITTER(-0.89),
        CONFETTI(-0.87),
        RED_SHOT(-0.85),
        BLUE_SHOT(-0.83),
        WHITE_SHOT(-0.81),
        SINELON_RAINBOW(-0.79),
        SINELON_PARTY(-0.77),
        SINELON_OCEAN(-0.75),
        SINELON_LAVA(-0.73),
        SINELON_FOREST(-0.71),
        BPM_RAINBOW(-0.69),
        BPM_OCEAN(-0.65),
        BPM_LAVA(-0.63),
        BPM_FOREST(-0.61),
        FIRE_MEDIUM(-0.59),
        FIRE_LARGE(-0.57),
        TWINKLES_RAINBOW(-0.55),
        TWINKLES_PARTY(-0.53),
        TWINKLES_OCEAN(-0.51),
        TWINKLES_LAVA(-0.49),
        TWINKLES_FOREST(-0.47),
        WAVES_RAINBOW(-0.45),
        WAVES_PARTY(-0.43),
        WAVES_OCEAN(-0.41),
        WAVES_LAVA(-0.39),
        WAVES_FOREST(-0.37),
        LARSON_RED(-0.35),
        LARSON_GRAY(-0.33),
        CHASE_RED(-0.31),
        CHASE_BLUE(-0.29),
        CHASE_GRAY(-0.27),
        HEARTBEAT_RED(-0.25),
        HEARTBEAT_BLUE(-0.23),
        HEARTBEAT_WHITE(-0.21),
        HEARTBEAT_GRAY(-0.19),
        BREATH_RED(-0.17),
        BREATH_BLUE(-0.15),
        BREATH_GRAY(-0.13),
        STROBE_BLUE(-0.09),
        STROBE_GOLD(-0.07),
        STROBE_WHITE(-0.05),
        COLOR1_BLEND_TO_BLACK(-0.03),
        COLOR1_LARSON(-0.01),
        COLOR1_CHASE(0.01),
        COLOR1_HEARTBEAT_SLOW(0.03),
        COLOR1_HEARTBEAT_MEDIUM(0.05),
        COLOR1_HEARTBEAT_FAST(0.07),
        COLOR1_BREATH_SLOW(0.09),
        COLOR1_BREATH_FAST(0.11),
        COLOR1_SHOT(0.13),
        COLOR1_STROBE(0.15),
        COLOR2_BLEND_TO_BLACK(0.17),
        COLOR2_LARSON(0.19),
        COLOR2_CHASE(0.21),
        COLOR2_HEARTBEAT_SLOW(0.23),
        COLOR2_HEARTBEAT_MEDIUM(0.25),
        COLOR2_HEARTBEAT_FAST(0.27),
        COLOR2_BREATH_SLOW(0.29),
        COLOR2_BREATH_FAST(0.31),
        COLOR2_SHOT(0.33),
        COLOR2_STROBE(0.35),
        SPARKLE_1_ON_2(0.37),
        SPARKLE_2_ON_1(0.39),
        GRADIENT_1_AND_2(0.41),
        BPM_1_AND_2(0.43),
        END_BLEND_1_AND_2(0.45),
        END_BLEND(0.47),
        COLOR_1_AND_2_NO_BLEND(0.49),
        TWINKLE_1_AND_2(0.51),
        WAVES_1_AND_2(0.53),
        SINELON_1_AND_2(0.55),
        HOT_PINK(0.57),
        DARK_RED(0.59),
        RED(0.61),
        RED_ORANGE(0.63),
        ORANGE(0.65),
        GOLD(0.67),
        YELLOW(0.69),
        LAWN_GREEN(0.71),
        LIME(0.73),
        DARK_GREEN(0.75),
        GREEN(0.77),
        BLUE_GREEN(0.79),
        AQUA(0.81),
        SKY_BLUE(0.83),
        DARK_BLUE(0.85),
        BLUE(0.87),
        BLUE_VIOLET(0.89),
        VIOLET(0.91),
        WHITE(0.93),
        GRAY(0.95),
        DARK_GRAY(0.97),
        BLACK(0.99);

        final double setpoint;

        ColorPattern(double setpoint) {
            this.setpoint = setpoint;
        }
    }

    /**
     * Allows for theming effects based on alliance color
     */
    public static class PatternTheme {
        public static final PatternTheme RAINBOW = new PatternTheme(ColorPattern.RAINBOW_LAVA, ColorPattern.RAINBOW_OCEAN, ColorPattern.RAINBOW);
        public static final PatternTheme BPM = new PatternTheme(ColorPattern.BPM_LAVA, ColorPattern.BPM_OCEAN, ColorPattern.BPM_OCEAN);
        public static final PatternTheme WAVES = new PatternTheme(ColorPattern.WAVES_LAVA, ColorPattern.WAVES_OCEAN, ColorPattern.WAVES_RAINBOW);
        public static final PatternTheme TWINKLES = new PatternTheme(ColorPattern.TWINKLES_LAVA, ColorPattern.TWINKLES_OCEAN, ColorPattern.TWINKLES_RAINBOW);
        public static final PatternTheme SINELON = new PatternTheme(ColorPattern.SINELON_LAVA, ColorPattern.SINELON_OCEAN, ColorPattern.SINELON_RAINBOW);
        public static final PatternTheme CHASE = new PatternTheme(ColorPattern.CHASE_RED, ColorPattern.CHASE_BLUE);
        public static final PatternTheme HEARTBEAT = new PatternTheme(ColorPattern.HEARTBEAT_RED, ColorPattern.HEARTBEAT_BLUE);
        public static final PatternTheme BREATH = new PatternTheme(ColorPattern.BREATH_RED, ColorPattern.BREATH_BLUE);
        public static final PatternTheme SOLID = new PatternTheme(ColorPattern.RED, ColorPattern.BLUE);
        public static final PatternTheme SHOT = new PatternTheme(ColorPattern.RED_SHOT, ColorPattern.BLUE_SHOT);


        private final ColorPattern red;
        private final ColorPattern blue;
        private final ColorPattern other;

        /**
         * Constructs a ColorScheme that depends on the current alliance
         *
         * @param redPattern     The pattern for the red alliance
         * @param bluePattern    The pattern for the blue alliance
         * @param defaultPattern The pattern to use if the robot doesn't know its alliance (like if not yet connected to the FMS)
         */
        public PatternTheme(ColorPattern redPattern, ColorPattern bluePattern, ColorPattern defaultPattern) {
            this.red = redPattern;
            this.blue = bluePattern;
            this.other = defaultPattern;
        }

        /**
         * Constructs a ColorScheme that depends on the current alliance and uses red as its default
         *
         * @param redPattern  The pattern for the red alliance
         * @param bluePattern The pattern for the blue alliance
         */
        public PatternTheme(ColorPattern redPattern, ColorPattern bluePattern) {
            this.red = redPattern;
            this.blue = bluePattern;
            this.other = redPattern;
        }

        /**
         * Constructs a ColorScheme that does not depend on the current alliance
         *
         * @param pattern The pattern to use for both alliances
         */
        public PatternTheme(ColorPattern pattern) {
            this.red = pattern;
            this.blue = pattern;
            this.other = pattern;
        }

        /**
         * Gets the {@link ColorPattern} for the current alliance, as reported by the FMS or driver station
         *
         * @return The themed pattern
         */
        public ColorPattern get() {
            switch (DriverStation.getAlliance()) {
                case Red:
                    return this.red;
                case Blue:
                    return this.blue;
                default:
                    return this.other;
            }
        }
    }
}
