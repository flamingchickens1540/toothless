package org.team1540.robot2022.commands.shooter;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class VelocitySetCommand extends CommandBase {
    private final Shooter shooter;
    private TalonFX flywheel;
    private double velocity; 
    public  VelocitySetCommand(Shooter shooter, /*LinearInterperlator interperlator,*/ TalonFX flywheel){
        this.shooter = shooter; 
        //this.interperlator = interperlator; 
        this.flywheel = flywheel; 
        addRequirements(shooter); 
    }
    
    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
                flywheel.set(TalonFXControlMode.Velocity, (velocity * 2048.0) / 600);
    }
}
