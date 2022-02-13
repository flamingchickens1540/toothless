// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team1540.robot2022;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.team1540.robot2022.commands.drivetrain.AutoTest;
import org.team1540.robot2022.commands.drivetrain.DriveTrain;
import org.team1540.robot2022.commands.drivetrain.PointToTarget;
import org.team1540.robot2022.commands.hood.Hood;
import org.team1540.robot2022.commands.hood.HoodSetCommand;
import org.team1540.robot2022.commands.intake.Intake;
import org.team1540.robot2022.commands.intake.IntakeFoldCommand;
import org.team1540.robot2022.commands.intake.IntakeSpinCommand;
import org.team1540.robot2022.utils.Limelight;
import org.team1540.robot2022.utils.ChickenSmartDashboard;
import org.team1540.robot2022.utils.NavX;
import org.team1540.robot2022.utils.RevBlinken;
import org.team1540.robot2022.utils.RevBlinken.GameStage;

import edu.wpi.first.wpilibj.DriverStation;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.PneumaticHub;

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
    // Hardware
    public final RevBlinken robotLEDs = new RevBlinken(0);
    public final Limelight limelight = new Limelight("limelight");
    public final NavX navx = new NavX(SPI.Port.kMXP);
    public final PneumaticHub ph = new PneumaticHub(Constants.ph);

    // Subsystems
    public final DriveTrain driveTrain = new DriveTrain(NeutralMode.Brake, navx);
    public final Hood hood = new Hood();
    public final Intake intake = new Intake();

    // Controllers
    public final XboxController driverController = new XboxController(0);
    public final XboxController copilotController = new XboxController(1);

    private SendableChooser<Command> autoChooser = new SendableChooser<>();
    public final InterpolationTable interpolationTable = new InterpolationTable();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        initSmartDashboard();
        configureButtonBindings();
        initModeTransitionBindings();
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
        // Driver
        new JoystickButton(driverController, Button.kX.value)
                .whenPressed(navx::zeroYaw);
        new JoystickButton(driverController, Button.kRightBumper.value)
                .whenHeld(new PointToTarget(driveTrain, limelight));

        // Copilot
        new JoystickButton(copilotController, Button.kA.value)
                .whenPressed(new HoodSetCommand(hood, true));
        new JoystickButton(copilotController, Button.kB.value)
                .whenPressed(new HoodSetCommand(hood, false));

        new JoystickButton(copilotController, Button.kX.value)
                .whenPressed(new IntakeFoldCommand(intake, true));
        new JoystickButton(copilotController, Button.kY.value)
                .whenPressed(new IntakeFoldCommand(intake, false));

        new JoystickButton(copilotController, Button.kLeftBumper.value)
                .whileHeld(new IntakeSpinCommand(intake, 0.5));
        new JoystickButton(copilotController, Button.kRightBumper.value)
                .whileHeld(new IntakeSpinCommand(intake, -0.5));
    }

    private void initModeTransitionBindings() {
        var autonomous = new Trigger(DriverStation::isAutonomousEnabled);
        var teleop = new Trigger(DriverStation::isTeleopEnabled);
        var disabled = new Trigger(DriverStation::isDisabled);

        teleop.whenActive(() -> {
            robotLEDs.applyPattern(DriverStation.getAlliance(), GameStage.TELEOP);
        });

        autonomous.whenActive(() -> {
            robotLEDs.applyPattern(DriverStation.getAlliance(), GameStage.AUTONOMOUS);
        });

        disabled.whenActive(() -> {
            robotLEDs.applyPattern(DriverStation.getAlliance(), GameStage.DISABLE);
        });
    }

    private void initSmartDashboard() {
        autoChooser.addOption("Test Auto", new AutoTest(driveTrain));
        SmartDashboard.putData(autoChooser);

        Shuffleboard.getTab("SmartDashboard")
                .add("NavX", navx)
                .withWidget(BuiltInWidgets.kGyro);

        // PointToTarget values
        ChickenSmartDashboard.putDefaultNumber("pointToTarget/kP", 0.7);
        ChickenSmartDashboard.putDefaultNumber("pointToTarget/kD", 0.4);
        SmartDashboard.putNumber("pointToTarget/pidOutput", 0);
        SmartDashboard.putNumber("pointToTarget/degreeDistanceToTarget", 0);
        ChickenSmartDashboard.putDefaultNumber("pointToTarget/pidClamp", 0.8);
        ChickenSmartDashboard.putDefaultNumber("pointToTarget/targetDeadzoneDegrees", 2);
        SmartDashboard.putBoolean("pointToTarget/isClamping", false);

        ChickenSmartDashboard.putDefaultNumber("ramsetePID/kP", 0.5);
        ChickenSmartDashboard.putDefaultNumber("tankDrive/maxVelocity", 0.8);
        ChickenSmartDashboard.putDefaultNumber("tankDrive/maxAcceleration", 0.5);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
