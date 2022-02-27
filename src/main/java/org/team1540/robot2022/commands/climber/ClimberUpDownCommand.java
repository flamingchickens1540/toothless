package org.team1540.robot2022.commands.climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimberUpDownCommand extends CommandBase {
    private final Climber climber;
    private final XboxController controller; // Copilot
    private final double deadzone = 0.3;

    public ClimberUpDownCommand(Climber climber, XboxController controller) {
        this.climber = climber;
        this.controller = controller;
        addRequirements(climber);
    }

    /**
     * Returns 0 if the input is within the deadzone, else the value
     *
     * @param value The joystick input
     * @return The value after deadzone is checked
     */
    private double applyDeadzone(double value) {
        if (Math.abs(value) <= deadzone)
            return 0;
        else
            return value;
    }

    public void execute() {
        climber.setLeftPercent(applyDeadzone(controller.getLeftY()));
        climber.setRightPercent(applyDeadzone(controller.getRightY()));
    }
}
