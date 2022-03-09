package org.team1540.robot2022.commands.indexer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.team1540.robot2022.commands.indexer.Indexer.IndexerState;
import org.team1540.robot2022.commands.shooter.Shooter;

/**
 * Eject the top ball out of the shooter
 */
public class EjectTopBallCommand extends SequentialCommandGroup {
    private final Shooter shooter;
    private final Indexer indexer;

    // TODO: The shooter doesn't spin up fast enough
    public EjectTopBallCommand(Indexer indexer, Shooter shooter) {
        this.indexer = indexer;
        this.shooter = shooter;

        double ballEjectSpeed = SmartDashboard.getNumber("indexer/ballEjectFlywheelRPM", 1000);

        addRequirements(indexer, shooter);
        addCommands(
                parallel( // Stop indexer and shooter
                        indexer.commandStop(),
                        shooter.commandStop()
                ),
                sequence(
                        new PrintCommand("Setting to " + ballEjectSpeed),
                        shooter.commandSetVelocity(ballEjectSpeed, ballEjectSpeed), // Run shooter at low velocity
                        new WaitCommand(2), // Wait for shooter spinup
                        indexer.commandSet(IndexerState.FORWARD, IndexerState.UNCHANGED) // Run top of indexer
                ),
                new WaitUntilCommand(() -> !indexer.getTopSensor()), // Wait until ball no longer seen by top sensor
                new WaitCommand(0.2) // Wait for ball to fully exit robot
        );
    }

    @Override
    public void end(boolean isInterrupted) {
        indexer.stop();
        shooter.stop();
    }
}
