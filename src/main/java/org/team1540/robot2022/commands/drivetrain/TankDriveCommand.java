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
    private final SlewRateLimiter leftRateLimiter = new SlewRateLimiter(DriveConstants.tankDriveAccelerationLimit);
    private final SlewRateLimiter rightRateLimiter = new SlewRateLimiter(DriveConstants.tankDriveAccelerationLimit);

    public TankDriveCommand(DriveTrain drivetrain, XboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        addRequirements(drivetrain);
    }


    /**
     * Caps the input value at the speed limit
     * @param value The speed to apply the limit to 
     * @return The capped value
     */
    public double applySpeedLimit(double value) {
        if (value > speedLimit) {
            return speedLimit;
        } else if (value < -speedLimit) {
            return -speedLimit;
        } else {
            return value;
        }
    }

    /**
     * Returns 0 if the input is within the deadzone and applies a SlewRateLimiter
     * @param value The joystick input
     * @param limiter The SlewRateLimiter to use
     * @return The value after deadzone is checked and SlewRateLimiter is applied
     */
    private double applyDeadzone(double value, SlewRateLimiter limiter) {
        if (Math.abs(value) <= deadzone) {
            limiter.reset(0);
            return 0;
        } else {
            return limiter.calculate(-value);
        }
    }

    /**
     * Applies the deadzone and speed limits to the input value
     * @param value The joystick input
     * @param limiter The SlewRateLimiter to use
     * @return The value after deadzone is checked, and acceleration and velocity are limited
     */
    private double applyLimits(double value, SlewRateLimiter limiter) {
        return applySpeedLimit(applyDeadzone(value, limiter));
    }

    public void execute() {
        double valueL = applyLimits(controller.getLeftY(), leftRateLimiter);
        double valueR = applyLimits(controller.getRightY(), rightRateLimiter);
        drivetrain.setPercent(valueL, valueR);
    }
}
