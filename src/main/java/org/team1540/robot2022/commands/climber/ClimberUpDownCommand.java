package org.team1540.robot2022.commands.climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2022.utils.MathUtils;

public class ClimberUpDownCommand extends CommandBase {
    private final Climber climber;
    private final XboxController controller; // Copilot
    private final double deadzone = 0.3;

    public ClimberUpDownCommand(Climber climber, XboxController controller) {
        this.climber = climber;
        this.controller = controller;
        addRequirements(climber);
    }

    public void execute() {
        climber.setLeftPercent(MathUtils.deadzone(controller.getLeftY(), deadzone));
        climber.setRightPercent(MathUtils.deadzone(controller.getRightY(), deadzone));
    }
}
