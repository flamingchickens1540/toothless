package org.team1540.robot2022;

import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.XboxController;

public class ShooterManualControl extends CommandBase {
  
private ShooterTesting shooter;
private final XboxController pilot = new XboxController(0); 

    public ShooterManualControl(ShooterTesting shooter, XboxController pilot){
        this.shooter = shooter;
        addRequirements(shooter);
        
    }

    public void execute(){
        if(pilot.getAButton()){
            shooter.setPercent(0.4);
        }
    }
 
}

