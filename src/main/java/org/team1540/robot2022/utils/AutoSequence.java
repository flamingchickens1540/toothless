package org.team1540.robot2022.utils;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team1540.robot2022.commands.drivetrain.Drivetrain;
import org.team1540.robot2022.utils.AutoHelper.AutoPath;

public class AutoSequence extends SequentialCommandGroup {

    public AutoPath[] paths = new AutoPath[0];

    /**
     * Stores paths in the auto sequence for highlighting
     *
     * @param paths The paths for this sequence
     */
    protected void addPaths(AutoPath... paths) {
        this.paths = paths;
    }

    /**
     * Highlights all of the paths in the sequence in red on Shuffleboard
     *
     * @param drivetrain The Drivetrain subsystem
     */
    public void highlightPaths(Drivetrain drivetrain) {
        drivetrain.fieldWidget.resetPathColors();
        for (AutoPath path : paths) {
            drivetrain.fieldWidget.setPathColor(path, "#FF0000FF");
        }
    }

}
