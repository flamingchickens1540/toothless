package org.team1540.robot2022.commands.drivetrain;

import org.team1540.robot2022.RamseteConfig;
import org.team1540.robot2022.commands.hood.Hood;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.commands.intake.Intake;
import org.team1540.robot2022.commands.intake.IntakeFoldCommand;
import org.team1540.robot2022.commands.intake.IntakeSpinCommand;
import org.team1540.robot2022.commands.shooter.ShootSequence;
import org.team1540.robot2022.commands.shooter.Shooter;
import org.team1540.robot2022.utils.Limelight;
import org.team1540.robot2022.utils.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Auto2BallSequence extends SequentialCommandGroup {

    /**
     * Constructs a new Auto2Ball Sequence
     * @param drivetrain The drivetrain subsystem (For driving)
     * @param intake The intake subsystem (For collecting balls)
     * @param indexer The indexer subsystem (For collecting balls and ShootSequence)
     * @param shooter The shooter subsystem (For ShootSequence)
     * @param limelight The limelight (For PointToTarget)
     * @param indexCommand The indexCommand (for cancelling and rescheduling)
     * @param isPosA If the robot is positioned in the upper right or bottom left starting position
     */
    public Auto2BallSequence(Drivetrain drivetrain, Intake intake, Indexer indexer, Shooter shooter, Hood hood, Limelight limelight, RepeatCommand indexCommand, boolean isPosA) {
        String trajectoryFile = "2ball.pos"+(isPosA?"A":"B")+".path1.wpilib.json";
        addCommands( 
            deadline( // End this command when the path sequence is done
                sequence (                          // Run the path sequence
                    RamseteConfig.getRamseteCommand(drivetrain, trajectoryFile),             // Path follow to collect first ball
                    new ShootSequence(shooter, indexer, drivetrain, hood, intake, limelight, indexCommand) // Shoot the 2 indexed balls (starts with one, collects one)
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
