package org.team1540.robot2022.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.team1540.robot2022.Constants;

/**
 * Moves the climber down until the current spikes
 */
public class ClimberZeroCommand extends SequentialCommandGroup {
    private final Climber climber;

    public ClimberZeroCommand(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
        addCommands(
                parallel(
                        // Zero left climber
                        sequence(
                                climber.commandSetPercentLeft(Constants.ClimberConstants.ZERO_DOWN_SPEED),
                                new WaitUntilCommand(() -> climber.getLeftCurrent() > SmartDashboard.getNumber("climber/currentLimit", Constants.ClimberConstants.ZERO_SPIKE_CURRENT)),
                                climber.commandSetPercentLeft(0)
                        ),

                        // Zero right climber
                        sequence(
                                climber.commandSetPercentRight(Constants.ClimberConstants.ZERO_DOWN_SPEED),
                                new WaitUntilCommand(() -> climber.getRightCurrent() > SmartDashboard.getNumber("climber/currentLimit", Constants.ClimberConstants.ZERO_SPIKE_CURRENT)),
                                climber.commandSetPercentRight(0)
                        )
                ),
                new InstantCommand(climber::enableLimits)
        );
    }

    @Override
    public void end(boolean isInterrupted) {
        climber.stop();
    }
}
