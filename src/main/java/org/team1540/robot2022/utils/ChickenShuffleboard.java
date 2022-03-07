package org.team1540.robot2022.utils;

import java.util.HashMap;
import java.util.Map;
import org.team1540.robot2022.Constants;
import org.team1540.robot2022.InterpolationTable;
import org.team1540.robot2022.RobotContainer;
import org.team1540.robot2022.commands.climber.Climber;
import org.team1540.robot2022.commands.drivetrain.Auto1BallSequence;
import org.team1540.robot2022.commands.drivetrain.Auto2BallSequence;
import org.team1540.robot2022.commands.drivetrain.Auto3BallSequence;
import org.team1540.robot2022.commands.drivetrain.Auto4BallSequence;
import org.team1540.robot2022.commands.drivetrain.Auto5BallSequence;
import org.team1540.robot2022.commands.drivetrain.Drivetrain;
import org.team1540.robot2022.commands.hood.Hood;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.commands.intake.Intake;
import org.team1540.robot2022.commands.shooter.ShootSequence;
import org.team1540.robot2022.commands.shooter.Shooter;
import org.team1540.robot2022.utils.AutoHelper.AutoPath;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public final class ChickenShuffleboard {
    public static final class DrivetrainTab {
        private static ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        public static SendableChooser<AutoSequence> autoChooser;

        
        private static ShuffleboardLayout tankDriveLimits = tab.getLayout("Tank Drive Limits", BuiltInLayouts.kList)
                .withPosition(5, 1)
                .withProperties(Map.of("labelPosition", "TOP"))
                .withSize(5, 2);

        public static NetworkTableEntry maxAcceleration = tankDriveLimits.addPersistent("Max Acceleration", 0.5)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 3))
                .getEntry();

        public static NetworkTableEntry maxVelocity = tankDriveLimits.addPersistent("Max Velocity", 1)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 3))
                .getEntry();

        private static ShuffleboardLayout encoders = tab.getLayout("Meters Traveled", BuiltInLayouts.kGrid)
                .withProperties(Map.of("numberOfRows", 1, "numberOfColumns", 2))
                .withPosition(0,3)
                .withSize(2,1);

        private static ShuffleboardLayout pidGrid = tab.getLayout("PID", BuiltInLayouts.kGrid)
                .withProperties(Map.of("numberOfRows", 2, "numberOfColumns", 2, "labelPosition", "TOP"))
                .withPosition(2,3)
                .withSize(2,2);

        private static ShuffleboardLayout pidError = pidGrid.getLayout("Error", BuiltInLayouts.kGrid)
                .withProperties(Map.of("numberOfRows", 1, "numberOfColumns", 2, "labelPosition", "TOP"))
                .withPosition(0,1)
                .withSize(2,1);

        public static NetworkTableEntry kP = pidGrid.addPersistent("kP", 0.3).withPosition(0, 0).withSize(2, 1).getEntry();


        private static void initialize(Drivetrain drivetrain, Intake intake, Indexer indexer,
                Shooter shooter, Hood hood) {

            autoChooser = new SendableChooser<>();
            autoChooser.addOption("1 Ball",          new Auto1BallSequence(drivetrain, intake, indexer, shooter, hood, false));
            autoChooser.addOption("1 Ball (Taxi)",   new Auto1BallSequence(drivetrain, intake, indexer, shooter, hood, true));
            autoChooser.setDefaultOption("2 Ball A", new Auto2BallSequence(drivetrain, intake, indexer, shooter, hood, true));
            autoChooser.addOption("2 Ball B",        new Auto2BallSequence(drivetrain, intake, indexer, shooter, hood, false));
            autoChooser.addOption("3 Ball",          new Auto3BallSequence(drivetrain, intake, indexer, shooter, hood));
            autoChooser.addOption("4 Ball",          new Auto4BallSequence(drivetrain, intake, indexer, shooter, hood));
            autoChooser.addOption("5 Ball",          new Auto5BallSequence(drivetrain, intake, indexer, shooter, hood));

            tab.add("Auto Selector", autoChooser)
                    .withPosition(5, 0)
                    .withSize(5, 1)
                    .withWidget(BuiltInWidgets.kSplitButtonChooser);

            encoders.addNumber("Left", drivetrain.driveLFront::getDistanceMeters).withPosition(0,0);
            encoders.addNumber("Right", drivetrain.driveRFront::getDistanceMeters).withPosition(1,0);

            pidError.addNumber("Left", drivetrain.driveLFront::getClosedLoopError).withPosition(0,0);
            pidError.addNumber("Right", drivetrain.driveRFront::getClosedLoopError).withPosition(1,0);

        }

        public static class Field {
            private static final String defaultColor = "#FFFFFFFF";

            public static final Field2d field2d = new Field2d();
            private static Map<String, Object> widgetProperties = new HashMap<>();
            private static final ComplexWidget widget = tab.add(field2d)
                    .withPosition(0, 0)
                    .withWidget(BuiltInWidgets.kField)
                    .withSize(5, 3)
                    .withProperties(widgetProperties);


            /**
             * Sets the color for a path
             * 
             * @param pathname the name of the path
             * @param color    the color to set it to (#RRGGBBAA)
             */
            public static void setPathColor(String pathname, String color) {
                widgetProperties.put(pathname, color);
                widget.withProperties(widgetProperties);
            }

            /**
             * Sets the color for a path
             * 
             * @param path  the path
             * @param color the color to set it to (#RRGGBBAA)
             */
            public static void setPathColor(AutoPath path, String color) {
                widgetProperties.put(path.name, color);
                widget.withProperties(widgetProperties);
            }


            /**
             * Resets all path colors to the default
             */
            public static void resetPathColors() {
                widgetProperties.forEach((key, value) -> {
                    setPathColor(key, defaultColor);
                });

            }

            /**
             * Adds a path to the field object
             * 
             * @param pathname   The name of the path
             * @param trajectory The path's trajectory
             */
            public static void addPath(String pathname, Trajectory trajectory) {
                field2d.getObject(pathname).setTrajectory(trajectory);
                setPathColor(pathname, defaultColor);
            }

            /**
             * Adds a path to the field object
             * 
             * @param path an AutoPath
             */
            public static void addPath(AutoPath path) {
                addPath(path.name, path.trajectory);
            }
        }
    }

    public static final class PointToTargetTab {
        private static ShuffleboardTab tab = Shuffleboard.getTab("Point To Target");
        private static ShuffleboardLayout pointPID = tab.getLayout("PID", BuiltInLayouts.kGrid)
                .withPosition(0, 0)
                .withSize(2, 2)
                .withProperties(Map.of("labelPosition", "TOP", "numberOfRows", 2,"numberOfColumns", 2));

        public static NetworkTableEntry kP = pointPID.addPersistent("kP", 0.7).withPosition(0, 0).getEntry();
        public static NetworkTableEntry kI = pointPID.addPersistent("kI", 0).withPosition(1, 0).getEntry();
        public static NetworkTableEntry kD = pointPID.addPersistent("kD", 0.4).withPosition(0, 1).getEntry();

        public static NetworkTableEntry pidClamp = pointPID.addPersistent("PID Clamp", 0.8).withPosition(1, 1).getEntry();

        public static NetworkTableEntry deadzone = tab.addPersistent("Target Deadzone Degrees", 0.8)
                .withWidget(BuiltInWidgets.kTextView)
                .withPosition(2, 0)
                .withSize(2, 1)
                .getEntry();

        public static NetworkTableEntry pidOutput = tab.add("PID Output", 0)
                .withPosition(2, 1)
                .getEntry();

        public static NetworkTableEntry distanceToTarget = tab.add("Distance to Target", 0)
                .withPosition(3, 1)
                .getEntry();

        public static NetworkTableEntry isClamping = tab.add("Is Clamping", false)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withPosition(4, 0)
                .getEntry();

        private static void initialize(Limelight limelight) {
            tab.addNumber("Calculated Distance", limelight::getCalculatedDistance).withPosition(5, 0).withSize(2,1);
            tab.addBoolean("Target Found", limelight::isTargetFound).withPosition(4, 1);
        }
    }

    public static class ShooterTab {
        private static ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
        
        private static ShuffleboardLayout velocity, error;

        public static NetworkTableEntry waitAfterFirstBall = tab.addPersistent("Wait After First Ball", 0.5)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 2))
                .withPosition(6, 0)
                .withSize(2, 1).getEntry();

        public static NetworkTableEntry targetError = tab.addPersistent("Target Error", 30)
                .withPosition(6,1)
                .withSize(2,1)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 100, "blockIncrement", 10))
                .getEntry();

        private static void initialize(Shooter shooter, ShootSequence shootSequence) {
            // PID Initialization
            Tuning.PIDs.initialize(shooter);
            shooter.updatePIDs();
            NetworkTableInstance.getDefault().getTable("Shuffleboard/Shooter").addEntryListener((table, key, entry, value, flags) -> shooter.updatePIDs(), EntryListenerFlags.kUpdate);

            // Motor Current
            tab.addNumber("Current", () -> shooter.shooterMotorFront.getStatorCurrent()+shooter.shooterMotorFront.getStatorCurrent()).withPosition(0, 2);

            // Velocities
            velocity = tab.getLayout("Velocity", BuiltInLayouts.kGrid)
                    .withPosition(1, 2)
                    .withSize(2, 1)
                    .withProperties(Map.of("labelPosition", "TOP", "numberOfRows", 1, "numberOfColumns", 2));
            
            // Motor RPMs    
            velocity.addNumber("Front", () -> shooter.shooterMotorFront.getVelocityRPM()).withPosition(0, 0);
            velocity.addNumber("Rear", () -> shooter.shooterMotorRear.getVelocityRPM()).withPosition(1, 0);
            
            // Errors
            error = tab.getLayout("Error", BuiltInLayouts.kGrid)
                    .withPosition(3, 2)
                    .withSize(3, 1)
                    .withProperties(Map.of("labelPosition", "TOP", "numberOfRows", 1, "numberOfColumns", 3));

            error.addNumber("Combined", shooter::getClosedLoopError).withPosition(0, 0);
            error.addNumber("Front", shooter::getFrontClosedLoopError).withPosition(1, 0);
            error.addNumber("Rear", shooter::getRearClosedLoopError).withPosition(2, 0);

            // Is spun up
            tab.addBoolean("Is Spun Up", shooter::isSpunUp)
                    .withPosition(4, 0)
                    .withWidget(BuiltInWidgets.kBooleanBox)
                    .withSize(2, 2);

            tab.addString("Profile", () -> shootSequence.profile+"").withPosition(0, 3).withSize(2,1);

            

    
        }
        public static class Tuning {
            private static ShuffleboardLayout tuning = tab.getLayout("Tuning", BuiltInLayouts.kList)
                    .withPosition(6, 2)
                    .withSize(2, 3);

            public static NetworkTableEntry enableManualSetpoints = tuning.add("Enabled", false)
                    .withPosition(0, 0)
                    .withWidget(BuiltInWidgets.kToggleSwitch)
                    .getEntry();
            
            public static NetworkTableEntry manualSetpointFront = tuning.add("Front", 1000).withPosition(0, 1).getEntry();
            public static NetworkTableEntry manualSetpointRear = tuning.add("Rear", 1000).withPosition(0, 2).getEntry();
            public static NetworkTableEntry manualSetpointHood = tuning.add("Hood", true).withWidget(BuiltInWidgets.kToggleButton).withPosition(0, 3).getEntry();

            public static class PIDs {
                private static ShuffleboardLayout frontPID = tab.getLayout("Front PID", BuiltInLayouts.kGrid)
                    .withPosition(0, 0)
                    .withSize(2, 2)
                    .withProperties(Map.of("labelPosition", "TOP", "numberOfRows", 2, "numberOfColumns", 2));
                
                public static NetworkTableEntry frontP;
                public static NetworkTableEntry frontI;
                public static NetworkTableEntry frontD;
                public static NetworkTableEntry frontF;


                private static ShuffleboardLayout rearPID = tab.getLayout("Rear PID", BuiltInLayouts.kGrid)
                    .withPosition(2, 0)
                    .withSize(2, 2)
                    .withProperties(Map.of("labelPosition", "TOP", "numberOfRows", 2, "numberOfColumns", 2));
                
                public static NetworkTableEntry rearP;
                public static NetworkTableEntry rearI;
                public static NetworkTableEntry rearD;
                public static NetworkTableEntry rearF;

                private static void initialize(Shooter shooter) {
                    frontP = frontPID.addPersistent("kP", shooter.frontP).withPosition(0, 0).getEntry();
                    frontI = frontPID.addPersistent("kI", shooter.frontI).withPosition(1, 0).getEntry();
                    frontD = frontPID.addPersistent("kD", shooter.frontD).withPosition(0, 1).getEntry();
                    frontF = frontPID.addPersistent("ff", shooter.frontF).withPosition(1, 1).getEntry();

                    rearP = rearPID.addPersistent("kP", shooter.rearP).withPosition(0, 0).getEntry();
                    rearI = rearPID.addPersistent("kI", shooter.rearI).withPosition(1, 0).getEntry();
                    rearD = rearPID.addPersistent("kD", shooter.rearD).withPosition(0, 1).getEntry();
                    rearF = rearPID.addPersistent("ff", shooter.rearF).withPosition(1, 1).getEntry();
                    
                }


            }
    }
        public static class Presets {
            private static ShuffleboardLayout presetList = tab.getLayout("Presets", BuiltInLayouts.kList);
            private static ShuffleboardLayout hubGrid     = presetList.getLayout("Hub",     BuiltInLayouts.kGrid).withProperties(Map.of("numberOfRows",1,"numberOfColumns", 2, "labelPosition", "TOP")).withPosition(0, 0);
            private static ShuffleboardLayout tarmacGrid  = presetList.getLayout("Tarmac",  BuiltInLayouts.kGrid).withProperties(Map.of("numberOfRows",1,"numberOfColumns", 2, "labelPosition", "TOP")).withPosition(0, 1);
            private static ShuffleboardLayout lowGoalGrid = presetList.getLayout("Lowgoal", BuiltInLayouts.kGrid).withProperties(Map.of("numberOfRows",1,"numberOfColumns", 2, "labelPosition", "TOP")).withPosition(0, 2);

            public static NetworkTableEntry hubFront = hubGrid.addPersistent("Front", InterpolationTable.hubFront).withPosition(0, 0).getEntry();
            public static NetworkTableEntry hubRear  = hubGrid.addPersistent("Rear", InterpolationTable.hubRear).withPosition(1, 0).getEntry();

            public static NetworkTableEntry tarmacFront = tarmacGrid.addPersistent("Front", InterpolationTable.tarmacFront).withPosition(0, 0).getEntry();
            public static NetworkTableEntry tarmacRear  = tarmacGrid.addPersistent("Rear", InterpolationTable.tarmacRear).withPosition(1, 0).getEntry();

            public static NetworkTableEntry lowGoalFront = lowGoalGrid.addPersistent("Front", InterpolationTable.lowGoalFront).withPosition(0, 0).getEntry();
            public static NetworkTableEntry lowGoalRear  = lowGoalGrid.addPersistent("Rear", InterpolationTable.lowGoalRear).withPosition(1, 0).getEntry();
        }
    }

    public static class ClimberTab {
        private static ShuffleboardTab tab = Shuffleboard.getTab("Climber");

        private static ShuffleboardLayout limits = tab.getLayout("Limits", BuiltInLayouts.kGrid).withPosition(0, 0).withSize(2, 1).withProperties(Map.of("numberOfRows", 1, "numberOfColumns", 2));
        public static NetworkTableEntry leftLimit = limits.addPersistent("Left", -470000).withPosition(0, 0).getEntry();
        public static NetworkTableEntry rightLimit = limits.addPersistent("Right", -470000).withPosition(1, 0).getEntry();

        private static ShuffleboardLayout encoders = tab.getLayout("Encoders", BuiltInLayouts.kGrid).withPosition(0, 1).withSize(2, 1).withProperties(Map.of("numberOfRows", 1, "numberOfColumns", 2));
        private static ShuffleboardLayout current = tab.getLayout("Current", BuiltInLayouts.kGrid).withPosition(2, 0).withSize(2, 2).withProperties(Map.of("numberOfRows", 2, "numberOfColumns", 1));
        private static ShuffleboardLayout statorCurrent = current.getLayout("Stator Current", BuiltInLayouts.kGrid).withPosition(0, 1).withSize(2, 1).withProperties(Map.of("numberOfRows", 1, "numberOfColumns", 2));

        public static NetworkTableEntry currentSpike = current.addPersistent("Zeroing Spike", Constants.ClimberConstants.ZERO_SPIKE_CURRENT).withPosition(0, 0).withSize(2, 1).getEntry();
    
        private static void initialize(Climber climber) {
            encoders.addNumber("Left", climber.motorLeft::getSelectedSensorPosition).withPosition(0, 0);
            encoders.addNumber("Right", climber.motorRight::getSelectedSensorPosition).withPosition(1, 0);

            statorCurrent.addNumber("Left", climber.motorLeft::getStatorCurrent).withPosition(0, 0);
            statorCurrent.addNumber("Right", climber.motorRight::getStatorCurrent).withPosition(1,0);

            
        }
    }

    public static class PnuematicsTab {
        private static ShuffleboardTab tab = Shuffleboard.getTab("Pnuematics");
        
        private static void initialize(PneumaticHub ph) {
            tab.addBoolean("Pressure Switch", ph::getPressureSwitch).withPosition(0, 0).withSize(2,2).withWidget(BuiltInWidgets.kBooleanBox);
            tab.addNumber("Pressure", () -> ph.getPressure(0)).withPosition(2, 0).withSize(2, 2).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("max", 120)); //TODO get right channel
        }
    }

    public static class IndexerTab {
        private static ShuffleboardTab tab = Shuffleboard.getTab("Indexer");
        
        public static NetworkTableEntry ballEjectFlywheelRPM = tab.addPersistent("Ball Eject Flywheel RPM", 1000).withPosition(3, 0).withSize(2, 1).getEntry();

        private static void initialize(Indexer indexer) {
            tab.addBoolean("Top Staged", indexer::getTopSensor).withPosition(0, 0);
            tab.addBoolean("Bottom Staged", indexer::getBottomSensor).withPosition(0, 1);
            tab.addBoolean("Is Full", indexer::isFull).withPosition(1, 0).withSize(2, 2);
            
        }
    }

    public static void initialize(RobotContainer robotContainer) {
        DrivetrainTab.initialize(robotContainer.drivetrain, robotContainer.intake, robotContainer.indexer, robotContainer.shooter, robotContainer.hood);
        ShooterTab.initialize(robotContainer.shooter, robotContainer.shootSequence);
        ClimberTab.initialize(robotContainer.climber);
        PnuematicsTab.initialize(robotContainer.ph);
        IndexerTab.initialize(robotContainer.indexer);
        PointToTargetTab.initialize(robotContainer.limelight);
        
    }
}
