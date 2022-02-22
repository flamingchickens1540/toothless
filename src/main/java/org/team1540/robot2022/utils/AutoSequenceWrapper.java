package org.team1540.robot2022.utils;

import org.team1540.robot2022.RamseteConfig;
import org.team1540.robot2022.commands.drivetrain.Drivetrain;
import org.team1540.robot2022.commands.indexer.IndexCommand;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.commands.intake.Intake;
import org.team1540.robot2022.commands.intake.IntakeFoldCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoSequenceWrapper {

    /**
     * Returns a SequentialCommandGroup that lowers the intake, then runs the intake and indexer
     * @param intake The intake subsystem
     * @param indexer The indexer subsystem
     * @return The SequentialCommandGroup
     */
    public static SequentialCommandGroup getIntakeSequence(Indexer indexer, Intake intake) {
        return new SequentialCommandGroup(
                new IntakeFoldCommand(intake, false),                // Lower the intake
                new WaitCommand(1),                                  // Wait for intake to fold down
                new RepeatCommand(new IndexCommand(indexer, intake)) // Run the indexer
        );
    }

    /**
     * Returns a ParallelDeadlineGroup that runs intake and indexer while following the given trajectory
     * @param drivetrain The drivetrain subsystem
     * @param intake The intake subsystem
     * @param indexer The indexer subsystem
     * @param trajectoryName the path of the trajectory file relative to `deploy/paths`
     * @return The SequentialCommandGroup
     */
    public static ParallelDeadlineGroup runPath(Drivetrain drivetrain, Intake intake, Indexer indexer, String trajectoryName) {
        return new ParallelDeadlineGroup(
                RamseteConfig.getRamseteCommand(drivetrain, trajectoryName),             // Path follow to collect first bal
                AutoSequenceWrapper.getIntakeSequence(indexer, intake)
        );
    }
}
