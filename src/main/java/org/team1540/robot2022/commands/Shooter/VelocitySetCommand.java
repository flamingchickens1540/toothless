package org.team1540.robot2022.commands.shooter;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.team1540.robot2022.utils.Limelight;
import org.team1540.robot2022.utils.LinearInterpolator;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class VelocitySetCommand extends CommandBase {
    private final Shooter shooter;
    private Limelight limeLight;
    private TalonFX flywheel;
    private double velocity;
    private LinearInterpolator interpolator; 
    public  VelocitySetCommand(Shooter shooter, LinearInterpolator interpolator, TalonFX flywheel, Limelight limelight){
        this.shooter = shooter; 
        this.interpolator = interpolator; 
        this.flywheel = flywheel; 
        this.limeLight = limeLight; 
        addRequirements(shooter); 
    }
    
    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
                flywheel.set(TalonFXControlMode.Velocity, (interpolator.getInterpolatedValue(limeLight.getCalculatedDistance()) * 2048.0) / 600);
    }
}
