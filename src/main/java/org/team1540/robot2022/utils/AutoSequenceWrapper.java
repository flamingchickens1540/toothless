package org.team1540.robot2022.utils;

import org.team1540.robot2022.RamseteConfig;
import org.team1540.robot2022.commands.drivetrain.Drivetrain;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.commands.intake.Intake;
import org.team1540.robot2022.commands.intake.IntakeSequence;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

public class AutoSequenceWrapper {



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
                new IntakeSequence(intake, indexer)
        );
    }
}
