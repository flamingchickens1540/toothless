package org.team1540.robot2022.commands.util;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2022.utils.FeatherClient;

/**
 * UpdateMatchInfo attempts to update the match ID and exits when it completed successfully
 */
public class UpdateMatchInfo extends CommandBase {
    private boolean updateSucceeded;

    public UpdateMatchInfo() {
    }

    @Override
    public void execute() {
        this.updateSucceeded = FeatherClient.updateMatchId();
    }

    @Override
    public boolean isFinished() {
        return updateSucceeded;
    }
}
