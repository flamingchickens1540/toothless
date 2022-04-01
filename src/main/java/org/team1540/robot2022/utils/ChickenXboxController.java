package org.team1540.robot2022.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public class ChickenXboxController extends XboxController {

    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is plugged into.
     */
    public ChickenXboxController(int port) {
        super(port);
    }


    /**
     * Switches the vibration between a value and 0 at an interval a specified number of times
     *
     * @param interval The time between each switch
     * @param value    The intensity to vibrate at (0 to 1)
     * @param count    How many times to turn on and off
     * @return A command group to do this
     */
    public Command rumblePulse(double interval, double value, int count) {
        LinkedList<CommandBase> commands = new LinkedList<>();
        commands.add(new PrintCommand("starting"));
        commands.add(commandSetRumble(value));
        commands.add(new WaitCommand(interval));
        commands.add(commandSetRumble(0));
//        System.out.println(commands);
        for (int i = 2; i <= count; i++) {
            CommandBase[] group = {
                    new WaitCommand(interval),
                    commandSetRumble(value),
                    new WaitCommand(interval),
                    commandSetRumble(0)
            };
            commands.addAll(List.of(group));
        }
        System.out.println(commands);
        Command[] cmdArray = commands.toArray(new Command[0]);
        System.out.println(Arrays.toString(cmdArray));
        return new SequentialCommandGroup(cmdArray);
    }

    /**
     * Set the rumble output for the Xbox controller on both hands.
     *
     * @param value The normalized value (0 to 1) to set the rumble to
     */
    public void setRumble(double value) {
        System.out.println(value);
        this.setRumble(RumbleType.kLeftRumble, value);
        this.setRumble(RumbleType.kRightRumble, value);
    }

    public ChickenInstantCommand commandSetRumble(double value) {
        return new ChickenInstantCommand(() -> this.setRumble(value), true);
    }
}
