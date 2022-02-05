package org.team1540.robot2022.commands.drivetrain;

import org.team1540.robot2022.Constants.DriveConstants;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TankDriveCommand extends CommandBase {
    private final double deadzone = 0.15;
    private final DriveTrain drivetrain;
    private final XboxController controller;
    private final double speedLimit = DriveConstants.tankDriveSpeedLimit;
    private final SlewRateLimiter leftRateLimiter = new SlewRateLimiter(DriveConstants.tankDriveAccellerationLimit);
    private final SlewRateLimiter rightRateLimiter = new SlewRateLimiter(DriveConstants.tankDriveAccellerationLimit);

    public TankDriveCommand(DriveTrain drivetrain, XboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        addRequirements(drivetrain);
    }

    public double applySpeedLimit(double value) {
        if (value > speedLimit) {
            return speedLimit;
        } else if (value < -speedLimit) {
            return -speedLimit;
        } else {
            return value;
        }
    }

    private double applyDeadzone(double value, SlewRateLimiter limiter) {
        if (Math.abs(value) <= deadzone) {
            limiter.reset(0);
            return 0;
        } else {
            return limiter.calculate(-value);
        }
    }

    private double applyLimits(double value, SlewRateLimiter limiter) {
        return applySpeedLimit(applyDeadzone(value, limiter));
    }

    public void execute() {
        double valueL = applyLimits(controller.getLeftY(), leftRateLimiter);
        double valueR = applyLimits(controller.getRightY(), rightRateLimiter);
        drivetrain.setPercent(valueL, valueR);
    }
}
