package org.team1540.robot2022.commands.shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterManualControl extends CommandBase {
    
    private Shooter shooter;
    private XboxController copilot; 
    
    public ShooterManualControl(Shooter shooter, XboxController copilot){
        this.shooter = shooter;
        this.copilot = copilot; 
        addRequirements(shooter);
        
    }
    
    @Override
    public void execute(){
        System.out.println("deploying correctly"); 
        if(copilot.getLeftBumper()){
            shooter.setVelocityRPMA(0.3);
        }
        if(copilot.getRightBumper()){
            shooter.setVelocityRPMB(0.3); 
        }
       
    } 
}

