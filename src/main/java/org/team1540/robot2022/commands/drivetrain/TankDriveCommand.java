package org.team1540.robot2022.commands.drivetrain;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TankDriveCommand extends CommandBase {
    private final double deadzone = 0.1;
    private final DriveTrain drivetrain;
    private final XboxController controller;
    private final double speedModifier = 0.5;

    public TankDriveCommand(DriveTrain drivetrain, XboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        addRequirements(drivetrain);
    }
    private double applyDeadzone(double value) {
        if (Math.abs(value) <= deadzone) {
            return 0;
        } else {
            return value;
        }
    }
    public void execute() {
        double valueL = speedModifier * applyDeadzone(controller.getLeftY());
        double valueR = speedModifier * applyDeadzone(controller.getRightY());
        drivetrain.setPercent(valueL, valueR);
    }
}
