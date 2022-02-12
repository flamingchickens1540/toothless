package org.team1540.robot2022.commands.shooter;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.team1540.robot2022.utils.InterpolationTable;
import org.team1540.robot2022.utils.Limelight;
import org.team1540.robot2022.utils.LinearInterpolator;

import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class VelocitySetCommand extends CommandBase {
    private final Shooter shooter;
    private Limelight limeLight;
    private double velocity;
    private InterpolationTable interpolationTable; 
    public  VelocitySetCommand(Shooter shooter, InterpolationTable interpolationTable, Limelight limelight){
        this.shooter = shooter; 
        this.interpolationTable = interpolationTable; 
        this.limeLight = limeLight; 
        addRequirements(shooter); 
    }
    
    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
                shooter.shooterMotorFront.set(TalonFXControlMode.Velocity, (interpolationTable.frontFlywheelInterpolator.getInterpolatedValue(limeLight.getCalculatedDistance()) * 2048.0) / 600);
                shooter.shooterMotorRear.set(TalonFXControlMode.Velocity, (interpolationTable.rearFlywheelInterpolator.getInterpolatedValue(limeLight.getCalculatedDistance()) * 2048.0) / 600);

    }
}
