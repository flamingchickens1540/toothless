package org.team1540.robot2022.utils;


import org.team1540.robot2022.commands.drivetrain.Drivetrain;
import org.team1540.robot2022.utils.AutoHelper.AutoPath;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;;

public class AutoSequence extends SequentialCommandGroup {

    public AutoPath[] paths = new AutoPath[0];

    /**
     * Stores paths in the auto sequence for highlighting
     * @param paths The paths for this sequence
     */
    protected void addPaths(AutoPath...paths) {
        this.paths = paths;
    }

    /**
     * Highlights all of the paths in the sequence in red on Shuffleboard
     * @param drivetrain The Drivetrain subsystem
     */
    public void highlightPaths(Drivetrain drivetrain) {
        ChickenShuffleboard.DrivetrainTab.Field.resetPathColors();
        for (AutoPath path : paths) {
            ChickenShuffleboard.DrivetrainTab.Field.setPathColor(path, "#FF0000FF");
        }
    }

}
