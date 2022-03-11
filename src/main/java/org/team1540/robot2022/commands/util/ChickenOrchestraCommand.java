package org.team1540.robot2022.commands.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.List;


public class ChickenOrchestraCommand extends SequentialCommandGroup {
    private final TalonFX[] instruments;

    /**
     * Constructs a command that plays a tone for a specified duration
     *
     * @param tone        The frequency to play (hz)
     * @param duration    How long the tone should last
     * @param instruments What motors the tone should be played on
     */
    public ChickenOrchestraCommand(double tone, double duration, TalonFX... instruments) {
        this.instruments = instruments;
        addCommands(
                new InstantCommand(() -> setAll(ControlMode.MusicTone, tone)),
                new WaitCommand(duration),
                new InstantCommand(() -> setAll(ControlMode.PercentOutput, 0))
        );
    }

    /**
     * Constructs a command that plays a .chrp file until it finishes
     *
     * @param musicFile   The name of the file to play, relative to the deploy directory
     * @param instruments The motors the music should be played on
     */
    public ChickenOrchestraCommand(String musicFile, TalonFX... instruments) {
        this.instruments = instruments;
        Orchestra orchestra = new Orchestra(List.of(instruments), musicFile);
        addCommands(
                new InstantCommand(orchestra::play),
                new WaitUntilCommand(() -> !orchestra.isPlaying()),
                new InstantCommand(orchestra::stop)
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        setAll(ControlMode.PercentOutput, 0);
    }


    /**
     * Sets all motors to a single tone
     *
     * @param mode  the ControlMode to use
     * @param value The value to set the motors to
     */
    private void setAll(ControlMode mode, double value) {
        for (TalonFX instrument : this.instruments) {
            instrument.set(mode, value);
        }
    }

}
