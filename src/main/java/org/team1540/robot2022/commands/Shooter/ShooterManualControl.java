package org.team1540.robot2022.commands.shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShooterManualControl extends CommandBase {
    
    private ShooterTesting shooter;
    private XboxController pilot; 
    
    public ShooterManualControl(ShooterTesting shooter, XboxController pilot){
        this.shooter = shooter;
        this.pilot = pilot; 
        addRequirements(shooter);
        
    }
    
    @Override
    public void execute(){
        System.out.println("deploying correctly"); 
        SmartDashboard.putNumber("leftY", pilot.getLeftY()); 
        shooter.setPercent(pilot.getLeftY());//setting speed of motors
    } 
}

