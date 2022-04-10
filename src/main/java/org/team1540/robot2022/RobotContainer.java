package org.team1540.robot2022;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.team1540.robot2022.commands.climber.ClimbSequence;
import org.team1540.robot2022.commands.climber.Climber;
import org.team1540.robot2022.commands.climber.ClimberUpDownCommand;
import org.team1540.robot2022.commands.climber.ClimberZeroCommand;
import org.team1540.robot2022.commands.drivetrain.*;
import org.team1540.robot2022.commands.hood.Hood;
import org.team1540.robot2022.commands.indexer.EjectBottomBallCommand;
import org.team1540.robot2022.commands.indexer.EjectTopBallCommand;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.commands.indexer.IndexerEjectCommand;
import org.team1540.robot2022.commands.intake.Intake;
import org.team1540.robot2022.commands.intake.IntakeSequence;
import org.team1540.robot2022.commands.shooter.ShootSequence;
import org.team1540.robot2022.commands.shooter.Shooter;
import org.team1540.robot2022.commands.vision.Vision;
import org.team1540.robot2022.utils.*;

public class RobotContainer {
    // Hardware
    public final RevBlinkin topLEDs = new RevBlinkin(9, true);
    public final RevBlinkin bottomLEDs = new RevBlinkin(8, false);
    public final Limelight limelight = new Limelight("limelight");
    public final NavX navx = new NavX(SPI.Port.kMXP);
    public final PneumaticHub ph = new PneumaticHub(Constants.PNEUMATIC_HUB);
    public final LIDAR lidar = new LIDAR(I2C.Port.kMXP);

    // Subsystems
    public final Drivetrain drivetrain = new Drivetrain(NeutralMode.Brake, navx, topLEDs);
    public final Hood hood = new Hood();
    public final Intake intake = new Intake();
    public final Indexer indexer = new Indexer(NeutralMode.Brake);
    public final Shooter shooter = new Shooter();
    public final Climber climber = new Climber();
    public final Vision vision = new Vision(drivetrain, navx, limelight);

    // Controllers
    public final ChickenXboxController driverController = new ChickenXboxController(0);
    public final ChickenXboxController copilotController = new ChickenXboxController(1);

    // Buttons
    public final DigitalInput zeroOdometry = new DigitalInput(0);

    // Commands
    public final IndexerEjectCommand indexerEjectCommand = new IndexerEjectCommand(indexer, intake);
    public final IntakeSequence intakeSequence = new IntakeSequence(intake, indexer, shooter);

    // coop:button(LJoystick,Left climber up/down,copilot)
    // coop:button(RJoystick,Right climber up/down,copilot)
    // coop:button(LTrigger,Climber up,copilot)
    // coop:button(RTrigger,Climber down,copilot)
    public final ClimberUpDownCommand climberUpDownCommand = new ClimberUpDownCommand(climber, copilotController);

    // coop:button(LJoystick,Left tank,pilot)
    // coop:button(RJoystick,Right tank,pilot)
    // coop:button(LTrigger,Drive Forward,pilot)
    // coop:button(RTrigger,Drive Backward,pilot)
    public final FFTankDriveCommand ffTankDriveCommand = new FFTankDriveCommand(drivetrain, driverController);

    public final ShootSequence shootSequence = new ShootSequence(shooter, indexer, drivetrain, hood, intake, vision, limelight, lidar, navx, Shooter.ShooterProfile.HUB, true, false, driverController);
    public final TestAllMotorsCommand testAllMotorsCommand = new TestAllMotorsCommand(drivetrain, intake, indexer, shooter, driverController);

    private final boolean ENABLE_COMPRESSOR = true;
    // Misc
    private final SendableChooser<AutoSequence> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        initSmartDashboard();
        configureButtonBindings();
        initModeTransitionBindings();
        DriverStation.silenceJoystickConnectionWarning(true);

//        FeatherClient.initialize();
        if (ENABLE_COMPRESSOR) {
            ph.enableCompressorDigital();
        } else {
            ph.disableCompressor();
        }
    }

    private void configureButtonBindings() {
        // Driver

        // coop:button(LBumper,Shoot HUB [hold],pilot)
        // coop:button(RBumper,Shoot FAR [hold],pilot)
        // coop:button(B,Shoot TESTING [press with bumper],pilot)
        new Trigger(driverController::getLeftBumper)
                .or(new Trigger(driverController::getRightBumper))
                .whileActiveOnce(new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            if (driverController.getBButton() && !DriverStation.isFMSAttached()) {
                                shootSequence.setProfile(Shooter.ShooterProfile.TESTING);
                            } else if (driverController.getLeftBumper()) {
                                shootSequence.setProfile(Shooter.ShooterProfile.HUB);
                            } else {
                                shootSequence.setProfile(Shooter.ShooterProfile.FAR);
                            }
                        }),
                        shootSequence
                ));


        // coop:button(DPadUp,Shoot from touching hub [press],pilot)
        new POVButton(driverController, DPadAxis.UP)
                .whenPressed(new InstantCommand(() -> shootSequence.setProfile(Shooter.ShooterProfile.HUB)));
        // coop:button(DPadDown,Shoot from outside tarmac [press],pilot)
        new POVButton(driverController, DPadAxis.DOWN)
                .whenPressed(new InstantCommand(() -> shootSequence.setProfile(Shooter.ShooterProfile.FAR)));
        // coop:button(DPadLeft,Decide shoot profile automatically [press],pilot)
        new POVButton(driverController, DPadAxis.LEFT)
                .whenPressed(new InstantCommand(() -> shootSequence.setProfile(Shooter.ShooterProfile.AUTOMATIC)));
        // coop:button(DPadRight,Shoot from low goal [press],pilot)
        new POVButton(driverController, DPadAxis.RIGHT)
                .whenPressed(new InstantCommand(() -> shootSequence.setProfile(Shooter.ShooterProfile.LOWGOAL)));

        // Copilot

        // coop:button(Y,Eject top ball [press],copilot)
        new JoystickButton(copilotController, Button.kY.value)
                .whenPressed(new EjectTopBallCommand(indexer, shooter));

        // coop:button(X,Eject bottom ball [press],copilot)
        new JoystickButton(copilotController, Button.kX.value)
                .whenPressed(new EjectBottomBallCommand(indexer, intake));


        // coop:button(DPadUp,Climber solenoids forward [press],copilot)
        new POVButton(copilotController, DPadAxis.UP)
                .whenPressed(new InstantCommand(() -> climber.setSolenoids(false)));
        // coop:button(DPadDown,Climber solenoids backward [press],copilot)
        new POVButton(copilotController, DPadAxis.DOWN)
                .whenPressed(new InstantCommand(() -> climber.setSolenoids(true)));


        // coop:button(DPadLeft,Lower intake [press],copilot)
        new POVButton(copilotController, DPadAxis.LEFT)
                .whenPressed(intake.commandSetFold(false));
        // coop:button(DPadRight,Raise intake [press],copilot)
        new POVButton(copilotController, DPadAxis.RIGHT)
                .cancelWhenPressed(intakeSequence)
                .whenPressed(intake.commandSetFold(true));

        // coop:button(A,Acquire balls [press],copilot)
        new JoystickButton(copilotController, Button.kA.value)
                .cancelWhenPressed(indexerEjectCommand)
                .whenPressed(intakeSequence);
        // coop:button(B,Stop intake and indexer [press],copilot)
        new JoystickButton(copilotController, Button.kB.value)
                .cancelWhenPressed(indexerEjectCommand)
                .cancelWhenPressed(intakeSequence);

        // coop:button(Back,Zero climber [press],copilot)
        new JoystickButton(copilotController, Button.kBack.value)
                .whenPressed(new ClimberZeroCommand(climber).andThen(new InstantCommand(climberUpDownCommand::schedule)));

        // coop:button(Start, Disable climber limits,copilot)
        new JoystickButton(copilotController, Button.kStart.value)
                .whenPressed(climber.commandDisableLimits());

        // coop:button(LBumper, Run climb sequence,copilot)
        new JoystickButton(copilotController, Button.kLeftBumper.value)
                .whenHeld(new ClimbSequence(climber, navx, topLEDs, copilotController, true)
                        .alongWith(commandSetLights(RevBlinkin.GameStage.ENDGAME)))
                .whenReleased(new InstantCommand(() -> {
                    if (!climberUpDownCommand.isScheduled()) {
                        climberUpDownCommand.schedule();
                    }
                }));

        // coop:button(RBumper, Run intake and indexer, copilot)
        new JoystickButton(copilotController, Button.kRightBumper.value)
                .whenHeld(intakeSequence)
                .whenReleased(intake.commandSetFold(true));

        // Robot hardware button
        new Trigger(zeroOdometry::get)
                .whenActive(new OdometryResetSequence(drivetrain, navx, vision, limelight, bottomLEDs));

        // SmartDashboard
        SmartDashboard.putData("ph/disableCompressor", new InstantCommand(ph::disableCompressor));
        SmartDashboard.putData("shooter/enableTestProfile", new InstantCommand(() -> shootSequence.setProfile(Shooter.ShooterProfile.TESTING)));
        SmartDashboard.putData("driveTrain/resetOdometry", OdometryResetSequence.getOdometryResetter(navx, drivetrain, vision));
    }

    private void initModeTransitionBindings() {
        Trigger enabled = new Trigger(RobotState::isEnabled);
        Trigger disabled = new Trigger(DriverStation::isDisabled);
        Trigger fmsConnected = new Trigger(DriverStation::isFMSAttached);

        Trigger autonomous = new Trigger(DriverStation::isAutonomousEnabled);
        Trigger teleop = new Trigger(DriverStation::isTeleopEnabled);
        Trigger endgame = new Trigger(() -> Timer.getMatchTime() <= 30).and(teleop);
//        Trigger endgamePrepare = new Trigger(() -> Timer.getMatchTime() <= 35).and(teleop); // TODO set to however long climb sequence takes + drive time

        Trigger indexerFull = new Trigger(indexer::isFull).and(teleop);


        fmsConnected.whenActive(bottomLEDs.commandSetPattern(RevBlinkin.ColorPattern.GREEN, false));

//        endgamePrepare.whenActive(() -> topLEDs.commandSetPattern(RevBlinkin.ColorPattern.VIOLET, false));

        endgame.whenActive(commandSetLights(RevBlinkin.GameStage.ENDGAME));


        // Turn lights gold when indexer is full
        indexerFull.whenActive(driverController.commandSetRumble(0.1).andThen(new WaitCommand(0.75)).andThen(driverController.commandSetRumble(0)));
        indexerFull.whenInactive(driverController.commandSetRumble(0));

        // Enable break mode when enabled
        enabled.whenActive(() -> {
            System.out.println("Setting brake mode");
            drivetrain.setNeutralMode(NeutralMode.Brake);
            intake.setNeutralMode(NeutralMode.Brake);
            indexer.setNeutralMode(NeutralMode.Brake);
            climber.setNeutralMode(NeutralMode.Brake);
        });

        // Disable break mode 2 seconds after disabling
        disabled.whenActive(new WaitCommand(2)
                .andThen(
                        new ConditionalCommand( // Check if the robot is still disabled to prevent enabling coast mode when the robot is enabled
                                new PrintCommand("Re-enabled, not coasting"),
                                new ChickenInstantCommand(() -> {
                                    System.out.println("Setting coast mode");
                                    drivetrain.setNeutralMode(NeutralMode.Coast);
                                    intake.setNeutralMode(NeutralMode.Coast);
                                    indexer.setNeutralMode(NeutralMode.Coast);
                                }, true),
                                RobotState::isEnabled)
                )
        );
    }

    private void initSmartDashboard() {
        autoChooser.addOption("1 Ball", new Auto1BallSequence(drivetrain, intake, indexer, vision, shooter, hood, limelight, lidar, navx, false));
        autoChooser.addOption("1 Ball (Taxi)", new Auto1BallSequence(drivetrain, intake, indexer, vision, shooter, hood, limelight, lidar, navx, true));
        autoChooser.addOption("2 Ball A", new Auto2BallSequence(drivetrain, intake, indexer, vision, shooter, hood, limelight, lidar, navx, true));
        autoChooser.addOption("2 Ball B", new Auto2BallSequence(drivetrain, intake, indexer, vision, shooter, hood, limelight, lidar, navx, false));
        autoChooser.addOption("2 Ball B (Mean)", new Auto2BallMeanSequence(drivetrain, intake, indexer, vision, shooter, hood, limelight, lidar, navx, true));
        autoChooser.addOption("2 Ball B (Defense)", new Auto2BallMeanSequence(drivetrain, intake, indexer, vision, shooter, hood, limelight, lidar, navx, false));
        autoChooser.setDefaultOption("3 Ball", new Auto3BallSequence(drivetrain, intake, indexer, vision, shooter, hood, limelight, lidar, navx));
        autoChooser.addOption("5 Ball", new Auto5BallSequence(drivetrain, intake, indexer, vision, shooter, hood, limelight, lidar, navx));

        SmartDashboard.putData("autoSelector", autoChooser);
        SmartDashboard.putData(CommandScheduler.getInstance());
        SmartDashboard.putData("zeroToStart", new ChickenInstantCommand(() -> drivetrain.resetOdometry(getAutonomousCommand().paths[0].trajectory.getInitialPose()), true));
        // Indexer values
        ChickenSmartDashboard.putDefaultNumber("intake/speed", 0.5);
        ChickenSmartDashboard.putDefaultNumber("indexer/waitDuration/top", 0.2);
        ChickenSmartDashboard.putDefaultNumber("indexer/waitDuration/bottom", 0.2);
        ChickenSmartDashboard.putDefaultNumber("indexer/ballEjectFlywheelRPM", 1000);

        // PointToTarget values
        ChickenSmartDashboard.putDefaultNumber("pointToTarget/kP", 0.7);
        ChickenSmartDashboard.putDefaultNumber("pointToTarget/kD", 0.4);
        ChickenSmartDashboard.putDefaultNumber("pointToTarget/navX_kP", 0.8);
        ChickenSmartDashboard.putDefaultNumber("pointToTarget/navX_kD", 0.5);
        ChickenSmartDashboard.putDebugBoolean("pointToTarget/turningWithLimelight", false);

        ChickenSmartDashboard.putDefaultNumber("pointToTarget/pidClamp", 0.8);
        ChickenSmartDashboard.putDefaultNumber("pointToTarget/targetDeadzoneDegrees", 2);

        ChickenSmartDashboard.putDebugNumber("pointToTarget/pidOutput", 0);
        ChickenSmartDashboard.putDebugNumber("pointToTarget/degreeDistanceToTarget", 0);

        ChickenSmartDashboard.putDebugBoolean("pointToTarget/isClamping", false);

        // Drivetrain values
        ChickenSmartDashboard.putDefaultNumber("ramsetePID/kP", 0.5);
        ChickenSmartDashboard.putDefaultNumber("drivetrain/tankDrive/maxVelocity", 1);
        ChickenSmartDashboard.putDefaultNumber("drivetrain/tankDrive/maxAcceleration", 0.5);
        ChickenSmartDashboard.putDefaultNumber("drivetrain/tankDrive/deadzone", 0.15);

        // Climber values
        ChickenSmartDashboard.putDefaultNumber("climber/PID/kP", 0.3);

        ChickenSmartDashboard.putDefaultNumber("shooter/tuning/frontRPM", -1000);
        ChickenSmartDashboard.putDefaultNumber("shooter/tuning/rearRPM", -1000);

        // Shoot when we're within this RPM from the target velocity (sum of both flywheel errors, plus or minus)
        ChickenSmartDashboard.putDefaultNumber("shooter/tuning/targetError", 30);

        // Highlight selected auto path
        getAutonomousCommand().highlightPaths(drivetrain);

        NetworkTableInstance.getDefault().getTable("SmartDashboard/autoSelector").addEntryListener((table, key, entry, value, flags) -> getAutonomousCommand().highlightPaths(drivetrain), EntryListenerFlags.kUpdate);
    }

    public AutoSequence getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public Command commandSetLights(RevBlinkin.GameStage stage) {
        return new InstantCommand(() -> {
            topLEDs.setPattern(stage);
            bottomLEDs.setPattern(stage);
        });
    }
}
