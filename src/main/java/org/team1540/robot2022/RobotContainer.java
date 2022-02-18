// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team1540.robot2022;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.team1540.robot2022.commands.drivetrain.AutoTest;
import org.team1540.robot2022.commands.drivetrain.DriveTrain;
import org.team1540.robot2022.commands.drivetrain.OdometryResetSequence;
import org.team1540.robot2022.commands.drivetrain.PointToTarget;
import org.team1540.robot2022.commands.drivetrain.TankDriveCommand;
import org.team1540.robot2022.commands.hood.Hood;
import org.team1540.robot2022.commands.hood.HoodSetCommand;
import org.team1540.robot2022.commands.indexer.IndexCommand;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.commands.indexer.IndexerEjectCommand;
import org.team1540.robot2022.commands.intake.Intake;
import org.team1540.robot2022.commands.intake.IntakeFoldCommand;
import org.team1540.robot2022.commands.intake.IntakeSpinCommand;
import org.team1540.robot2022.utils.ChickenSmartDashboard;
import org.team1540.robot2022.utils.Limelight;
import org.team1540.robot2022.utils.NavX;
import org.team1540.robot2022.utils.RepeatCommand;
import org.team1540.robot2022.utils.RevBlinken;
import org.team1540.robot2022.utils.RevBlinken.GameStage;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
    public final Indexer indexer = new Indexer(NeutralMode.Brake);

    // Controllers
    public final XboxController driverController = new XboxController(0);
    public final XboxController copilotController = new XboxController(1);

    // Buttons
    public final DigitalInput zeroOdometry = new DigitalInput(0);
    
    // Commands
    public final RepeatCommand indexCommand = new RepeatCommand(new IndexCommand(indexer));
    public final IndexerEjectCommand indexerEjectCommand = new IndexerEjectCommand(indexer);
    public final TankDriveCommand tankDriveCommand = new TankDriveCommand(driveTrain, driverController);

    // Misc
    public final InterpolationTable interpolationTable = new InterpolationTable();
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();



    

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        initSmartDashboard();
        configureButtonBindings();
        initModeTransitionBindings();
        DriverStation.silenceJoystickConnectionWarning(false);
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
        new JoystickButton(copilotController, Button.kX.value)
                // .cancelWhenPressed(indexerEjectCommand)
                .whenPressed(indexCommand);
                
        
        new JoystickButton(copilotController, Button.kY.value)
                .cancelWhenPressed(indexCommand)
                .whenPressed(indexerEjectCommand);
                

        new JoystickButton(copilotController, Button.kA.value)
                .cancelWhenPressed(indexerEjectCommand)
                .cancelWhenPressed(indexCommand);


        new JoystickButton(copilotController, Button.kLeftBumper.value)
                .whileHeld(new IntakeSpinCommand(intake, Constants.IntakeConstants.speed));
        new JoystickButton(copilotController, Button.kRightBumper.value)
                .whileHeld(new IntakeSpinCommand(intake, -Constants.IntakeConstants.speed));

        // Button

        new Trigger(zeroOdometry::get)
                .whenActive(new OdometryResetSequence(driveTrain, navx, limelight));

        //SmartDashboard
        SmartDashboard.putData(new IntakeFoldCommand(intake, true));
        SmartDashboard.putData(new IntakeFoldCommand(intake, false));

        SmartDashboard.putData(new HoodSetCommand(hood, true));
        SmartDashboard.putData(new HoodSetCommand(hood, false));

        SmartDashboard.putData(new InstantCommand(ph::disableCompressor));
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
        SmartDashboard.putData(CommandScheduler.getInstance());

        Shuffleboard.getTab("SmartDashboard")
                .add("NavX", navx)
                .withWidget(BuiltInWidgets.kGyro);


        // Indexer values
        ChickenSmartDashboard.putDefaultNumber("intake/speed", 0.5);
        ChickenSmartDashboard.putDefaultNumber("indexer/waitDuration/top", 0.2);
        ChickenSmartDashboard.putDefaultNumber("indexer/waitDuration/bottom", 0.2);

        // PointToTarget values
        ChickenSmartDashboard.putDefaultNumber("pointToTarget/kP", 0.7);
        ChickenSmartDashboard.putDefaultNumber("pointToTarget/kD", 0.4);
        
        ChickenSmartDashboard.putDefaultNumber("pointToTarget/pidClamp", 0.8);
        ChickenSmartDashboard.putDefaultNumber("pointToTarget/targetDeadzoneDegrees", 2);

        SmartDashboard.putNumber("pointToTarget/pidOutput", 0);
        SmartDashboard.putNumber("pointToTarget/degreeDistanceToTarget", 0);
        
        SmartDashboard.putBoolean("pointToTarget/isClamping", false);

        // Drivetrain values
        ChickenSmartDashboard.putDefaultNumber("ramsetePID/kP", 0.5);
        ChickenSmartDashboard.putDefaultNumber("tankDrive/maxVelocity", 0.8);
        ChickenSmartDashboard.putDefaultNumber("tankDrive/maxAcceleration", 0.5);

        SmartDashboard.putNumber("shooter/tarmacDefaultFrontRPM", 1000);
        SmartDashboard.putNumber("shooter/tarmacDefaultRearRPM", 1000);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
