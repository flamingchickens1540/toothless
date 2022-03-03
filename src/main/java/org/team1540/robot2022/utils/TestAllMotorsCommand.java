package org.team1540.robot2022.utils;

import org.team1540.robot2022.Constants;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TestAllMotorsCommand extends CommandBase {
    private XboxController controller;

    private final ChickenTalonFX[] allTalonFXs = new ChickenTalonFX[]{
        new ChickenTalonFX(Constants.DriveConstants.Motors.LEFT_FRONT),
        new ChickenTalonFX(Constants.DriveConstants.Motors.LEFT_REAR),
        new ChickenTalonFX(Constants.DriveConstants.Motors.RIGHT_FRONT),
        new ChickenTalonFX(Constants.DriveConstants.Motors.RIGHT_REAR),
        new ChickenTalonFX(Constants.IntakeConstants.FALCON),
        new ChickenTalonFX(Constants.ShooterConstants.FRONT),
        new ChickenTalonFX(Constants.ShooterConstants.REAR),
        new ChickenTalonFX(Constants.IndexerConstants.IndexerMotors.TOP_MOTOR),
        new ChickenTalonFX(Constants.IndexerConstants.IndexerMotors.BOTTOM_MOTOR),
    };

    private final SendableChooser<Integer> motorChooser = new SendableChooser<>();

    public TestAllMotorsCommand(XboxController controller) {
        this.controller = controller;

        motorChooser.setDefaultOption("Drive: Left Front", 0);
        motorChooser.addOption("Drive: Left Rear", 1);
        motorChooser.addOption("Drive: Right Front", 2);
        motorChooser.addOption("Drive: Right Rear", 3);
        motorChooser.addOption("Intake: Main", 4);
        motorChooser.addOption("Shooter: Front", 5);
        motorChooser.addOption("Shooter: Rear", 6);
        motorChooser.addOption("Indexer: Top", 7);
        motorChooser.addOption("Indexer: bottom", 8);

        SmartDashboard.putData(motorChooser);
    }

    public void execute() {
        double percentOutput = MathUtils.deadzone(controller.getLeftTriggerAxis(), 0.15);
        percentOutput -= MathUtils.deadzone(controller.getRightTriggerAxis(), 0.15);
        if (percentOutput < -1) {
            percentOutput = -1;
        } else if (percentOutput > 1) {
            percentOutput = 1;
        }
        allTalonFXs[motorChooser.getSelected()].setPercent(percentOutput);
    }

    public void end(boolean isInterrupted) {
        allTalonFXs[motorChooser.getSelected()].setPercent(0);
    } 
}
