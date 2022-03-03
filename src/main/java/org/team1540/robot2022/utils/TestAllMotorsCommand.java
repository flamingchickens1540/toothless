package org.team1540.robot2022.utils;

import org.team1540.robot2022.Constants;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestAllMotorsCommand extends CommandBase {
    private XboxController controller;
    private final SendableChooser<ChickenTalonFX> motorChooser = new SendableChooser<>();

    public TestAllMotorsCommand(Drivetrain drivetrain, Intake intake, Indexer indexer, Shooter shooter, XboxController controller) {
        this.controller = controller;

        addRequirements(drivetrain);
        addRequirements(intake);
        addRequirements(indexer);
        addRequirements(shooter);

        motorChooser.setDefaultOption("Drive: Left Front", drivetrain.driveLFront);
        motorChooser.addOption("Drive: Left Rear", drivetrain.driveLRear);
        motorChooser.addOption("Drive: Right Front", drivetrain.driveRFront);
        motorChooser.addOption("Drive: Right Rear", drivetrain.driveRRear);
        motorChooser.addOption("Intake: Main", intake.motor);
        motorChooser.addOption("Shooter: Front", shooter.shooterMotorFront);
        motorChooser.addOption("Shooter: Rear", shooter.shooterMotorRear);
        motorChooser.addOption("Indexer: Top", indexer.topMotor);
        motorChooser.addOption("Indexer: bottom", indexer.bottomMotor);

        SmartDashboard.putData(motorChooser);
    }

    public void execute() {
        double percentOutput = MathUtils.deadzone(controller.getLeftTriggerAxis(), 0.15);
        percentOutput -= MathUtils.deadzone(controller.getRightTriggerAxis(), 0.15);
        motorChooser.getSelected().setPercent(percentOutput);
    }

    public void end(boolean isInterrupted) {
        motorChooser.getSelected().setPercent(0);
    } 
}
