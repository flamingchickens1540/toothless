package org.team1540.robot2022.commands.climber;

import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2022.utils.ChickenXboxController;
import org.team1540.robot2022.utils.NavX;
import org.team1540.robot2022.utils.RevBlinkin;

public class ClimbSequence extends ParallelCommandGroup {
    private static final double SWING_THRESHOLD_LOWER = 27;
    private static final double SWING_THRESHOLD_UPPER = 23;
    private Climber climber;
    private NavX navx;

    private ChickenXboxController controller;
    private double lastRoll;

    public ClimbSequence(Climber climber, NavX navx, RevBlinkin lights, ChickenXboxController controller, boolean toTraversal) {
        this.climber = climber;
        this.navx = navx;
        this.controller = controller;
        addCommands(
                new InstantCommand(() -> controller.setRumble(0.1)),
                sequence(
                        // Mid Bar
                        runUntilSpike(1, 35), // Retract arms as far as they can go
                        new WaitCommand(0.2),

                        // High Bar
                        climber.commandSetPercent(-1), // Start extending arms (static hooks attached)
                        new WaitCommand(0.2),
                        new InstantCommand(() -> climber.setSolenoids(true)), // Retract arm pnuematics

                        raiseUntilLimit(-1),                                // Raise arms to maximum height

                        lights.commandSetPattern(RevBlinkin.ColorPattern.YELLOW, true),
                        new WaitUntilCommand(this::isOptimalSwing),           // Wait for robot to be swinging in the right place
                        lights.commandSetPattern(RevBlinkin.ColorPattern.GREEN, true),
                        new InstantCommand(() -> climber.setSolenoids(false)),// Move arms to hit high bar
                        new WaitCommand(1),                                   // Wait for arms to finish moving TODO can we lower this?
                        runUntilSpike(1, 35),                                // Retract the arms as far as they can go

                        // Traversal Bar
                        new ConditionalCommand(
                                sequence(
                                        new WaitCommand(0.5),
                                        climber.commandSetPercent(-1), // Start extending arms (static hooks attached)
                                        new WaitCommand(0.2),
                                        new InstantCommand(() -> climber.setSolenoids(true)), // Retract arm pnuematics

                                        raiseUntilLimit(-1),                                // Raise arms to maximum height
                                        new WaitUntilCommand(this::isOptimalSwing),           // Wait for robot to be swinging in the right place
                                        new InstantCommand(() -> climber.setSolenoids(false)),// Move arms to hit traversal bar
                                        new WaitCommand(1),                                   // Wait for arms to finish moving TODO can we lower this?
                                        runUntilSpike(1, 35)                                // Retract the arms as far as they can go
                                ),
                                new PrintCommand(""),
                                () -> toTraversal)
                )

        );
        addRequirements(climber);
    }

    @Override
    public void end(boolean interrupted) {

        climber.stop();
        controller.setRumble(0);
    }

    /**
     * Returns if the robot is at an optimal swing for transferring bars.
     *
     * @return true if the robot is at an optimal angle and swinging the right direction
     */
    private boolean isOptimalSwing() {
        double roll = navx.getRoll();
        if (roll - lastRoll <= 0 && (SWING_THRESHOLD_UPPER <= roll)) { //Check if robot is swinging the right way TODO make sure swing isn't inverted, tune threshold and range.
            lastRoll = roll;
            return true;
        }
        lastRoll = roll;
        return false;
    }

    private CommandGroupBase runUntilSpike(double speed, double spikeCurrent) {
        return parallel(
                sequence(
                        climber.commandSetPercentLeft(speed),
                        new WaitUntilCommand(() -> climber.getLeftCurrent() > spikeCurrent),
                        climber.commandSetPercentLeft(0)
                ),
                sequence(
                        climber.commandSetPercentRight(speed),
                        new WaitUntilCommand(() -> climber.getRightCurrent() > spikeCurrent),
                        climber.commandSetPercentRight(0)
                ));
    }

    private CommandGroupBase raiseUntilLimit(double speed) {
        return parallel(
                sequence(
                        climber.commandSetPercentLeft(speed),
                        new WaitUntilCommand(() -> !climber.sensorLeft.get()),
                        climber.commandSetPercentLeft(0)
                ),
                sequence(
                        climber.commandSetPercentRight(speed),
                        new WaitUntilCommand(() -> !climber.sensorRight.get()),
                        climber.commandSetPercentRight(0)
                )
        );
    }
}
