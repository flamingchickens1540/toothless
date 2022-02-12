package org.team1540.robot2022.commands.intake;

import org.team1540.robot2022.commands.intake.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class IntakeUpCommand extends CommandBase {
    private final Intake intake; 
    public IntakeUpCommand(Intake intake){
        this.intake = intake;  
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
    }
}
