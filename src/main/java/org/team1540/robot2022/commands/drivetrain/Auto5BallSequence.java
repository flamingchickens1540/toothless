package org.team1540.robot2022.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.team1540.robot2022.commands.hood.Hood;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.commands.intake.Intake;
import org.team1540.robot2022.commands.intake.IntakeSequence;
import org.team1540.robot2022.commands.shooter.ShootSequence;
import org.team1540.robot2022.commands.shooter.Shooter;
import org.team1540.robot2022.commands.vision.Vision;
import org.team1540.robot2022.utils.*;
import org.team1540.robot2022.utils.AutoHelper.AutoPath;


public class Auto5BallSequence extends AutoSequence {
    public Auto5BallSequence(Drivetrain drivetrain, Intake intake, Indexer indexer, Vision vision, Shooter shooter, Hood hood, Limelight limelight, LIDAR lidar, NavX navx) {
        addPaths(AutoPath.auto2Ball1A, AutoPath.auto5Ball2, AutoPath.auto5Ball3, AutoPath.auto5Ball4);
        addCommands(
                AutoHelper.runPathWithSpinup(drivetrain, intake, indexer, shooter, hood, AutoPath.auto2Ball1A), // Follow path to collect ball 2
                drivetrain.lights.commandSetPattern(RevBlinkin.ColorPattern.RED),
                new ShootSequence(shooter, indexer, drivetrain, hood, intake, vision, limelight, lidar, navx, Shooter.ShooterProfile.FAR, false, true),
                drivetrain.lights.commandSetPattern(RevBlinkin.ColorPattern.VIOLET),
                AutoHelper.runPathWithSpinup(drivetrain, intake, indexer, shooter, hood, AutoPath.auto5Ball2), // Follow path to collect balls 3 and 4// If (2 balls are indexed) then shoot, else keep going. In case human player ball isn't present
                new ShootSequence(shooter, indexer, drivetrain, hood, intake, vision, limelight, lidar, navx, Shooter.ShooterProfile.FAR, false, false),

                AutoHelper.runPath(drivetrain, intake, indexer, shooter, AutoPath.auto5Ball3), // Follow path to collect ball 5
                deadline(
                        new WaitUntilCommand(indexer::isFull).withTimeout(1),
                        new IntakeSequence(intake, indexer)
                ),
                AutoHelper.runPathWithSpinup(drivetrain, intake, indexer, shooter, hood, AutoPath.auto5Ball4), // Follow path to collect ball 5
                new ShootSequence(shooter, indexer, drivetrain, hood, intake, vision, limelight, lidar, navx, Shooter.ShooterProfile.FAR, true, true)
        );
    }
}

