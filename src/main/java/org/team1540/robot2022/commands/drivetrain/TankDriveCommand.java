package org.team1540.robot2022.commands.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TankDriveCommand extends CommandBase {
    private final double deadzone = 0.15;
    private final DriveTrain drivetrain;
    private final XboxController controller;
    private final SlewRateLimiter leftRateLimiter = new SlewRateLimiter(SmartDashboard.getNumber("tankDrive/maxAcceleration", 0.5));
    private final SlewRateLimiter rightRateLimiter = new SlewRateLimiter(SmartDashboard.getNumber("tankDrive/maxAcceleration", 0.5));

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
        double speedLimit = SmartDashboard.getNumber("tankDrive/maxVelocity", 0.8);
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
        // Reverse controls so it drives intake-first
        double valueR = -applyLimits(controller.getLeftY(), leftRateLimiter);
        double valueL = -applyLimits(controller.getRightY(), rightRateLimiter);
        drivetrain.setPercent(valueL, valueR);
    }
}
