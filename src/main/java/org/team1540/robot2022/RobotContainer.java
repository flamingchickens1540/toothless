// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team1540.robot2022;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.team1540.robot2022.commands.Intake.Intake;
import org.team1540.robot2022.commands.drivetrain.AutoTest;
import org.team1540.robot2022.commands.drivetrain.DriveTrain;
import org.team1540.robot2022.utils.NavX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import main.java.org.team1540.robot2022.commands.shooter.setVelocity;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    //public XboxController pilot = new XboxController(0);
    public XboxController copilot = new XboxController(1); 
    //private ShooterTesting shooter = new ShooterTesting();
    public Intake intake = new Intake(copilot);

    public Shooter shooter = new Shooter(copilot); 

    public final DriveTrain driveTrain;

    public final XboxController driverController = new XboxController(0);

    public final NavX navx = new NavX(SPI.Port.kMXP);

    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        driveTrain = new DriveTrain(NeutralMode.Brake, navx);

        initSmartDashboard();
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        new JoystickButton(driverController, Button.kX.value)
                .whenPressed(() -> navx.zeroYaw());
        new JoystickButton(copilot, Button.kLeftBumper.value)
                .whenHeld(new SetVelocity(shooter)); 
    }

    private void initSmartDashboard() {
        autoChooser.addOption("Test Auto", new AutoTest(driveTrain));
        SmartDashboard.putData(autoChooser);

        Shuffleboard.getTab("SmartDashboard")
            .add("NavX", navx)
            .withWidget(BuiltInWidgets.kGyro);

        SmartDashboard.putNumber("drivePID/kP", SmartDashboard.getNumber("drivePID/kP", 0.5));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
