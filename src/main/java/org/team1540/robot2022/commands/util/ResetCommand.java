package org.team1540.robot2022.commands.util;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import org.team1540.robot2022.commands.climber.Climber;
import org.team1540.robot2022.commands.climber.ClimberZeroCommand;
import org.team1540.robot2022.commands.hood.Hood;
import org.team1540.robot2022.commands.intake.Intake;


public class ResetCommand extends SequentialCommandGroup {
    private final Climber climber;
    private final Hood hood;
    private final Intake intake;

    public ResetCommand(Climber climber, Hood hood, Intake intake, PneumaticHub ph) {
        this.climber = climber;
        this.hood = hood;
        this.intake = intake;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.climber, this.hood, this.intake);
        addCommands(
                new PrintCommand("starting reset"),
                parallel(
                        new PrintCommand("starting parallel"),
                        new ClimberZeroCommand(climber),
                        new InstantCommand(() -> climber.setSolenoids(true)),
                        new InstantCommand(() -> hood.set(false)),
                        new InstantCommand(() -> intake.setFold(true))
                ),
                new WaitUntilCommand(() -> !ph.getPressureSwitch()),
                new PrintCommand("Done!!!!!!!")
        );


    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("ending reset");
        super.end(interrupted);
    }
}
