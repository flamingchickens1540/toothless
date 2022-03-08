package org.team1540.robot2022;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import org.team1540.robot2022.commands.climber.Climber;
import org.team1540.robot2022.commands.climber.ClimberUpDownCommand;
import org.team1540.robot2022.commands.climber.ClimberZeroCommand;
import org.team1540.robot2022.commands.drivetrain.Drivetrain;
import org.team1540.robot2022.commands.drivetrain.FFTankDriveCommand;
import org.team1540.robot2022.commands.drivetrain.OdometryResetSequence;
import org.team1540.robot2022.commands.drivetrain.PointToTarget;
import org.team1540.robot2022.commands.hood.Hood;
import org.team1540.robot2022.commands.indexer.EjectBottomBallCommand;
import org.team1540.robot2022.commands.indexer.EjectTopBallCommand;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.commands.indexer.IndexerEjectCommand;
import org.team1540.robot2022.commands.intake.Intake;
import org.team1540.robot2022.commands.intake.IntakeSequence;
import org.team1540.robot2022.commands.shooter.ShootSequence;
import org.team1540.robot2022.commands.shooter.Shooter;
import org.team1540.robot2022.utils.AutoSequence;
import org.team1540.robot2022.utils.ChickenShuffleboard;
import org.team1540.robot2022.utils.DPadAxis;
import org.team1540.robot2022.utils.LIDAR;
import org.team1540.robot2022.utils.Limelight;
import org.team1540.robot2022.utils.NavX;
import org.team1540.robot2022.utils.RevBlinken;
import org.team1540.robot2022.utils.TestAllMotorsCommand;
import org.team1540.robot2022.utils.RevBlinken.GameStage;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    private final boolean ENABLE_COMPRESSOR = true;

    // Hardware
    public final RevBlinken robotLEDs = new RevBlinken(0);
    public final Limelight limelight = new Limelight("limelight");
    public final NavX navx = new NavX(SPI.Port.kMXP);
    public final PneumaticHub ph = new PneumaticHub(Constants.PNEUMATIC_HUB);
    public final LIDAR lidar = new LIDAR(I2C.Port.kMXP);

    // Subsystems
    public final Drivetrain drivetrain = new Drivetrain(NeutralMode.Brake, navx);
    public final Hood hood = new Hood();
    public final Intake intake = new Intake();
    public final Indexer indexer = new Indexer(NeutralMode.Brake);
    public final Shooter shooter = new Shooter();
    public final Climber climber = new Climber();

    // Controllers
    public final XboxController driverController = new XboxController(0);
    public final XboxController copilotController = new XboxController(1);

    // Buttons
    public final DigitalInput zeroOdometry = new DigitalInput(0);

    // Commands
    public final IndexerEjectCommand indexerEjectCommand = new IndexerEjectCommand(indexer, intake);
    public final IntakeSequence intakeSequence = new IntakeSequence(intake, indexer, shooter);
    public final ShootSequence shootSequence = new ShootSequence(shooter, indexer, drivetrain, hood, intake, limelight, lidar, navx, Shooter.ShooterProfile.HUB, true);

    // coop:button(LJoystick,Left climber up/down,copilot)
    // coop:button(RJoystick,Right climber up/down,copilot)
    // coop:button(LTrigger,Climber up,copilot)
    // coop:button(RTrigger,Climber down,copilot)
    public final ClimberUpDownCommand climberUpDownCommand = new ClimberUpDownCommand(climber, copilotController);

    // coop:button(LJoystick,Left tank,pilot)
    // coop:button(RJoystick,Right tank,pilot)
    // coop:button(LTrigger,Forward,pilot)
    // coop:button(RTrigger,Reverse,pilot)
    public final FFTankDriveCommand ffTankDriveCommand = new FFTankDriveCommand(drivetrain, driverController);

    // Unsure what buttons to assign to this, currently uses triggers when called.
    public final TestAllMotorsCommand testAllMotorsCommand = new TestAllMotorsCommand(drivetrain, intake, indexer, shooter, driverController);


    public RobotContainer() {
        ChickenShuffleboard.initialize(this);

        initSmartDashboard();
        configureButtonBindings();
        initModeTransitionBindings();
        DriverStation.silenceJoystickConnectionWarning(true);

        if (ENABLE_COMPRESSOR) {
            ph.enableCompressorDigital();
        } else {
            ph.disableCompressor();
        }
    }

    private void configureButtonBindings() {
        // Driver

        // coop:button(LBumper,Shoot [hold],pilot)
        new JoystickButton(driverController, Button.kLeftBumper.value)
                .whenHeld(shootSequence);

        // coop:button(LBumper,Shoot [hold],pilot)
        new Trigger(() -> driverController.getLeftTriggerAxis() == 1)
                .whileActiveOnce(shootSequence);

        // coop:button(RBumper,Point to target [hold],pilot)
        new JoystickButton(driverController, Button.kRightBumper.value)
                .whenHeld(new PointToTarget(drivetrain, limelight, navx));

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
        // coop:button(RBumper,Outtake all through indexer [hold],copilot)
        new JoystickButton(copilotController, Button.kRightBumper.value)
                .cancelWhenPressed(intakeSequence)
                .whileHeld(indexerEjectCommand);
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

        // Robot hardware button
        new Trigger(zeroOdometry::get)
                .whenActive(new OdometryResetSequence(drivetrain, navx, limelight));

    }

    private void initModeTransitionBindings() {
        var enabled = new Trigger(RobotState::isEnabled);
        var disabled = new Trigger(DriverStation::isDisabled);
        var autonomous = new Trigger(DriverStation::isAutonomousEnabled);
        var teleop = new Trigger(DriverStation::isTeleopEnabled);

        teleop.whenActive(() -> robotLEDs.applyPattern(DriverStation.getAlliance(), GameStage.TELEOP));
        autonomous.whenActive(() -> robotLEDs.applyPattern(DriverStation.getAlliance(), GameStage.AUTONOMOUS));
        disabled.whenActive(() -> robotLEDs.applyPattern(DriverStation.getAlliance(), GameStage.DISABLE));

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
                                new InstantCommand(),
                                new InstantCommand(() -> {
                                    System.out.println("Setting coast mode");
                                    drivetrain.setNeutralMode(NeutralMode.Coast);
                                    intake.setNeutralMode(NeutralMode.Coast);
                                    indexer.setNeutralMode(NeutralMode.Coast);
                                    climber.setNeutralMode(NeutralMode.Coast);
                                }),
                                RobotState::isEnabled)
                )
        );
    }

    private void initSmartDashboard() {
        
        SmartDashboard.putData(CommandScheduler.getInstance());
        
        // Highlight selected auto path
        getAutonomousCommand().highlightPaths(drivetrain);
        
        NetworkTableInstance.getDefault().getTable("Shuffleboard/Drivetrain/Auto Selector").addEntryListener((table, key, entry, value, flags) -> getAutonomousCommand().highlightPaths(drivetrain), EntryListenerFlags.kUpdate);

        
    }

    public AutoSequence getAutonomousCommand() {
        return ChickenShuffleboard.DrivetrainTab.autoChooser.getSelected();
    }
}
