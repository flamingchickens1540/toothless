/*package org.team1540.robot2022;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PneumaticsTest extends SubsystemBase{
    private final XboxController pilot;
    Solenoid PH = new Solenoid(PneumaticsModuleType.REVPH, 0); 
    Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
    XboxController xboxController = new XboxController(0); 

    public void pneumaticsTest(XboxController pilot){
        this.pilot = pilot; 
    }
    
    @Override
    public void periodic() {
        if(pilot.getAButton())
        {
              PH.set(true);
         }
         if(pilot.getBButton())
         {
              PH.set(false);
          }
       
       
       
       
       /* phCompressor.enableDigital();
        phCompressor.disable();
        boolean enabled = phCompressor.enabled();
        boolean pressureSwitch = phCompressor.getPressureSwitchValue();
        double current = phCompressor.getCurrent();
        if (xboxController.getYButtonPressed()) {
            PH.toggle();
             }
        //SmartDashboard.putData("open solenoid", new SolenoidCommand()); 
        */ 
    /*}  

}
*/ 


