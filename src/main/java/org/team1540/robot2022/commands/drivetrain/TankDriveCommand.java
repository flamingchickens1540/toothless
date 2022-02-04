package org.team1540.robot2022.commands.drivetrain;

import org.team1540.robot2022.Constants.DriveConstants;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TankDriveCommand extends CommandBase {
    private final double deadzone = 0.1;
    private final DriveTrain drivetrain;
    private final XboxController controller;
    private final double speedModifier = 1;
    private final SlewRateLimiter leftRateLimiter = new SlewRateLimiter(DriveConstants.tankDriveAccellerationLimit);
    private final SlewRateLimiter rightRateLimiter = new SlewRateLimiter(DriveConstants.tankDriveAccellerationLimit);

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
        double valueL = leftRateLimiter.calculate(-speedModifier * applyDeadzone(controller.getLeftY()));
        double valueR = rightRateLimiter.calculate(-speedModifier * applyDeadzone(controller.getRightY()));
        drivetrain.setPercent(valueL, valueR);
    }
}
