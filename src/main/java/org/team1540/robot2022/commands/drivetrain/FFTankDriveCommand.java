package org.team1540.robot2022.commands.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2022.utils.ChickenSmartDashboard;
import org.team1540.robot2022.utils.MathUtils;

public class FFTankDriveCommand extends CommandBase {
    private double deadzone = 0.15;

    private final Drivetrain drivetrain;
    private final XboxController controller;

    private SlewRateLimiter slewRateLimiter;

    public FFTankDriveCommand(Drivetrain drivetrain, XboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        addRequirements(drivetrain);
    }

    private double scaleStickToVelocity(double stickValue) {
        return MathUtils.normalize(stickValue, -1, 1, drivetrain.getMinVelocity(), drivetrain.getMaxVelocity());
    }

    @Override
    public void initialize() {
        slewRateLimiter = new SlewRateLimiter(SmartDashboard.getNumber("drivetrain/tankDrive/maxAcceleration", 2));
    }

    @Override
    public void execute() {
        double triggers = MathUtils.deadzone(controller.getLeftTriggerAxis(), 0.05) - MathUtils.deadzone(controller.getRightTriggerAxis(), 0.05);
//        double triggers = 0;
        ChickenSmartDashboard.putDebugNumber("drivetrain/tankDrive/debug/triggers", triggers);

        deadzone = SmartDashboard.getNumber("drivetrain/tankDrive/deadzone", 0.15);
        double leftThrottle = scaleStickToVelocity(MathUtils.deadzone(controller.getLeftY(), deadzone) + triggers);
        double rightThrottle = scaleStickToVelocity(MathUtils.deadzone(controller.getRightY(), deadzone) + triggers);

        ChickenSmartDashboard.putDebugNumber("drivetrain/tankDrive/debug/leftStick", controller.getLeftY());
        ChickenSmartDashboard.putDebugNumber("drivetrain/tankDrive/debug/rightStick", controller.getRightY());
        ChickenSmartDashboard.putDebugNumber("drivetrain/tankDrive/debug/leftThrottle", leftThrottle);
        ChickenSmartDashboard.putDebugNumber("drivetrain/tankDrive/debug/rightThrottle", rightThrottle);
        double combined = Math.abs(leftThrottle) + Math.abs(rightThrottle);


        double ratioL = leftThrottle / combined;
        double ratioR = rightThrottle / combined;

        ChickenSmartDashboard.putDebugNumber("drivetrain/tankDrive/debug/combined", combined);
        double total = slewRateLimiter.calculate(combined / 2) * 2.0;
        ChickenSmartDashboard.putDebugNumber("drivetrain/tankDrive/debug/total", total);

        double leftCalculated = ratioL * total;
        double rightCalculated = ratioR * total;

        ChickenSmartDashboard.putDebugNumber("drivetrain/tankDrive/debug/calc/left", leftCalculated);
        ChickenSmartDashboard.putDebugNumber("drivetrain/tankDrive/debug/calc/right", rightCalculated);

        drivetrain.setFFVelocity(rightCalculated, leftCalculated);

//        if (leftCalculated + rightCalculated == 0) {
//            // Slow down instead of stopping immediately when inputs are 0
//            drivetrain.setFFVelocity(total / 2, total / 2);
//        } else {
//            // This is reversed to make the intake be the front of the robot
//            drivetrain.setFFVelocity(rightCalculated, leftCalculated);
//        }


    }
}
