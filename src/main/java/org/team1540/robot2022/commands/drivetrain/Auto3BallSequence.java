package org.team1540.robot2022.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.team1540.robot2022.InterpolationTable;
import org.team1540.robot2022.RamseteConfig;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.commands.intake.Intake;
import org.team1540.robot2022.commands.intake.IntakeFoldCommand;
import org.team1540.robot2022.commands.intake.IntakeSpinCommand;
import org.team1540.robot2022.commands.shooter.ShootSequence;
import org.team1540.robot2022.commands.shooter.Shooter;
import org.team1540.robot2022.utils.Limelight;
import org.team1540.robot2022.utils.RepeatCommand;

public class Auto3BallSequence extends SequentialCommandGroup {

    public Auto3BallSequence(Drivetrain drivetrain, Intake intake, Indexer indexer, Shooter shooter, Limelight limelight, RepeatCommand indexCommand, InterpolationTable interpolationTable) {
        addCommands(
                deadline( // End this command when the path sequence is done
                        sequence(                          // Run the path sequence
                                RamseteConfig.getRamseteCommand(drivetrain, "2ball.posA.path1.wpilib.json"), // Path follow to collect first ball
                                new ShootSequence(shooter, indexer, drivetrain, limelight, indexCommand, interpolationTable),     // Shoot the 2 indexed balls (starts with one, collects one)
                                RamseteConfig.getRamseteCommand(drivetrain, "3ball.posA.path2.wpilib.json"), // Path follow to collect second ball
                                new ShootSequence(shooter, indexer, drivetrain, limelight, indexCommand, interpolationTable)      // Shoot the 1 indexed balls (collects)
                        ),
                        sequence(
                                new IntakeFoldCommand(intake, false), // Lower the intake
                                new WaitCommand(1),                   // Wait for intake to fold down
                                new IntakeSpinCommand(intake, 0.5)    // Spin the intake
                        ),
                        indexCommand                        // Run the indexer
                )
        );
    }
}
