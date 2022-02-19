package org.team1540.robot2022.commands.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TankDriveCommand extends CommandBase {
    private final double deadzone = 0.15;
    private final DriveTrain drivetrain;
    private final XboxController controller;
    private SlewRateLimiter rateLimiter = new SlewRateLimiter(
            SmartDashboard.getNumber("driveTrain/tankDrive/maxAcceleration", 0.5));

    public TankDriveCommand(DriveTrain drivetrain, XboxController controller) {
        NetworkTableInstance.getDefault()
                .getEntry("SmartDashboard/driveTrain/tankDrive/maxAcceleration")
                .addListener(this::limitUpdateListener, EntryListenerFlags.kUpdate);
        this.drivetrain = drivetrain;
        this.controller = controller;
        addRequirements(drivetrain);
    }

    /**
     * Converts the input percent to a meter per second value
     * 
     * @param percent The percent input
     * 
     * @return The speed in meters per second
     */
    public double calculateMPS(double inputPercent) {
        double percent;

        if (inputPercent > 1)
            percent = 1;
        else if (inputPercent < -1)
            percent = -1;
        else
            percent = inputPercent;
        double maximumSpeed = SmartDashboard.getNumber("driveTrain/tankDrive/maxVelocity", 1);
        return percent * maximumSpeed;
    }

    /**
     * Returns 0 if the input is within the deadzone, else the value
     * 
     * @param value The joystick input
     * 
     * @return The value after deadzone is checked
     */
    private double applyDeadzone(double value) {
        if (Math.abs(value) <= deadzone)
            return 0;
        else
            return value;
    }

    private double applyLimiter(double value) {
        if (value == 0) {
            rateLimiter.reset(0);
            return 0;
        } else {
            return rateLimiter.calculate(value);
        }
    }

    /**
     * Applies the deadzone and converts inputs to meters per second
     * 
     * @param value The joystick input
     * 
     * @return The value after deadzone is checked and the units are converted
     */
    private double applyJoystickModifiers(double value) {
        double percentOutput = applyDeadzone(value) - controller.getLeftTriggerAxis()
                + controller.getRightTriggerAxis();
        return calculateMPS(percentOutput);
    }

    private void limitUpdateListener(EntryNotification event) {
        rateLimiter = new SlewRateLimiter(event.value.getDouble());
    }

    public void execute() {
        // Reverse controls so it drives intake-first, remove negatives and swap left
        // and right to drive shooter first

        double valueR = applyJoystickModifiers(controller.getLeftY());
        double valueL = applyJoystickModifiers(controller.getRightY());
        SmartDashboard.putNumber("driveTrain/tankDrive/debug/valueL", valueL);
        SmartDashboard.putNumber("driveTrain/tankDrive/debug/valueR", valueR);
        
        double combined = Math.abs(valueR) + Math.abs(valueL);

        double ratioL = valueL / combined;
        double ratioR = valueR / combined;
        SmartDashboard.putNumber("driveTrain/tankDrive/debug/ratioL", ratioL);
        SmartDashboard.putNumber("driveTrain/tankDrive/debug/ratioR", ratioR);

        double total = applyLimiter(combined / 2) * 2.0;
        SmartDashboard.putNumber("driveTrain/tankDrive/debug/total", total);

        double leftCalculated = ratioL * total;
        double rightCalculated = ratioR * total;
        SmartDashboard.putNumber("driveTrain/tankDrive/debug/calcL", leftCalculated);
        SmartDashboard.putNumber("driveTrain/tankDrive/debug/calcR", rightCalculated);
        
        drivetrain.setPercent(leftCalculated, rightCalculated);
    }
}
