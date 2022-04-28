package org.team1540.robot2022.commands.drivetrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ChildDriveCommand extends CommandBase {
    private final Drivetrain drivetrain;
    private final Joystick controller;

    public ChildDriveCommand(Drivetrain drivetrain, Joystick controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double leftPercent, rightPercent;
        rightPercent = leftPercent = controller.getY();
        rightPercent -= controller.getX() * (0.7);
        leftPercent += controller.getX() * (0.7);
        double scaleFactor = (controller.getZ() - 1) * -0.18;
        drivetrain.setPercent(leftPercent * scaleFactor, rightPercent * scaleFactor);

    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setPercent(0, 0);
    }
}
