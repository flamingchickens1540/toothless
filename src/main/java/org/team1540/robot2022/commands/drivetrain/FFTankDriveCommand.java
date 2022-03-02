package org.team1540.robot2022.commands.drivetrain;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2022.utils.MathUtils;

public class FFTankDriveCommand extends CommandBase {
    private final double deadzone = 0.15;

    private final Drivetrain drivetrain;
    private final XboxController controller;

    public FFTankDriveCommand(Drivetrain drivetrain, XboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        addRequirements(drivetrain);
    }

    private double scaleStickToVelocity(double stickValue) {
        return MathUtils.normalize(stickValue, -1, 1, drivetrain.getMinVelocity(), drivetrain.getMaxVelocity());
    }

    public void execute() {
        double triggers = MathUtils.deadzone(controller.getLeftTriggerAxis(), deadzone) - MathUtils.deadzone(controller.getRightTriggerAxis(), deadzone);
        double leftThrottle = MathUtils.deadzone(controller.getLeftY(), deadzone) + triggers;
        double rightThrottle = MathUtils.deadzone(controller.getRightY(), deadzone) + triggers;

        // This is reversed to make the intake be the front of the robot
        drivetrain.setFFVelocity(scaleStickToVelocity(rightThrottle), scaleStickToVelocity(leftThrottle));
    }
}
