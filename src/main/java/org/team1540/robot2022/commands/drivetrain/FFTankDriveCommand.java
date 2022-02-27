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
        return MathUtils.normalize(stickValue, 0, 1, drivetrain.getMinVelocity(), drivetrain.getMaxVelocity());
    }

    public void execute() {
        double leftThrottle = MathUtils.deadzone(controller.getLeftY(), deadzone);
        double rightThrottle = MathUtils.deadzone(controller.getRightY(), deadzone);
        drivetrain.setFFVelocity(scaleStickToVelocity(leftThrottle), scaleStickToVelocity(rightThrottle));
    }
}
