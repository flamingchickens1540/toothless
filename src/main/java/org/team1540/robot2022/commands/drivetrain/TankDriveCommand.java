package org.team1540.robot2022.commands.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2022.utils.ChickenShuffleboard;
import org.team1540.robot2022.utils.MathUtils;

public class TankDriveCommand extends CommandBase {
    private final double deadzone = 0.15;
    private final Drivetrain drivetrain;
    private final XboxController controller;
    private SlewRateLimiter rateLimiter = new SlewRateLimiter(ChickenShuffleboard.DrivetrainTab.maxAcceleration.getDouble(0.5));

    public TankDriveCommand(Drivetrain drivetrain, XboxController controller) {
        NetworkTableInstance.getDefault()
                .getEntry("Preferences/drivetrain/tankDrive/maxAcceleration")
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
        double maximumSpeed = ChickenShuffleboard.DrivetrainTab.maxVelocity.getDouble(1);
        return percent * maximumSpeed;
    }

    private double applyLimiter(double value) {
        return rateLimiter.calculate(value);
    }

    /**
     * Applies the deadzone and converts inputs to meters per second
     * 
     * @param value The joystick input
     * 
     * @return The value after deadzone is checked and the units are converted
     */
    private double applyJoystickModifiers(double value) {
        double percentOutput = MathUtils.deadzone(value, deadzone) - controller.getLeftTriggerAxis()
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
        
        double combined = Math.abs(valueR) + Math.abs(valueL);

        double ratioL = valueL / combined;
        double ratioR = valueR / combined;

        double total = applyLimiter(combined / 2) * 2.0;

        double leftCalculated = ratioL * total;
        double rightCalculated = ratioR * total;
        
        drivetrain.setPercent(leftCalculated, rightCalculated);
    }
}
