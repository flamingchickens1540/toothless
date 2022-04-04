// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team1540.robot2022;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.team1540.robot2022.commands.climber.ClimberUpDownCommand;
import org.team1540.robot2022.commands.climber.ClimberZeroCommand;
import org.team1540.robot2022.commands.util.ResetCommand;
import org.team1540.robot2022.utils.RevBlinkin;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    private RobotContainer robotContainer;

    private Command autonomousCommand;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        this.robotContainer = new RobotContainer();

        robotContainer.limelight.setLeds(false);
        robotContainer.bottomLEDs.setPattern(RevBlinkin.ColorPattern.YELLOW);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        // Update the limelight's custom SmartDashboard values
        robotContainer.limelight.updateSmartDashboardValues();
        robotContainer.lidar.updateSmartDashboardValues();

        SmartDashboard.putBoolean("pneumatics/pressureSwitch", robotContainer.ph.getPressureSwitch());
        SmartDashboard.putString("shooter/profile", robotContainer.shootSequence.profile + "");

        SmartDashboard.putNumber("navx/pitch", robotContainer.navx.getPitch());
        SmartDashboard.putNumber("navx/roll", robotContainer.navx.getRoll());

        SmartDashboard.putBoolean("climber/sensor/left", robotContainer.climber.sensorLeft.get());
        SmartDashboard.putBoolean("climber/sensor/right", robotContainer.climber.sensorRight.get());
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        robotContainer.limelight.setLeds(false);
        robotContainer.topLEDs.setPattern(RevBlinkin.GameStage.DISABLE);
        robotContainer.bottomLEDs.setPattern(RevBlinkin.GameStage.DISABLE);
        robotContainer.driverController.setRumble(0);
        robotContainer.copilotController.setRumble(0);

        System.out.println("Disabled");
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
        robotContainer.topLEDs.setPattern(RevBlinkin.GameStage.AUTONOMOUS);
        robotContainer.bottomLEDs.setPattern(RevBlinkin.GameStage.AUTONOMOUS);

        new ClimberZeroCommand(robotContainer.climber).schedule();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
        robotContainer.topLEDs.setPattern(RevBlinkin.GameStage.TELEOP);
        robotContainer.bottomLEDs.setPattern(RevBlinkin.GameStage.TELEOP);

        robotContainer.drivetrain.setDefaultCommand(robotContainer.ffTankDriveCommand);

        new ClimberUpDownCommand(robotContainer.climber, robotContainer.copilotController).schedule();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        System.out.println("Test enabled");
        LiveWindow.setEnabled(false);
        new ResetCommand(robotContainer.climber, robotContainer.hood, robotContainer.intake, robotContainer.ph).schedule();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }


}
