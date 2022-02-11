
package org.team1540.robot2022.commands.hood;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
    private XboxController copilot;
    Solenoid PH = new Solenoid(PneumaticsModuleType.REVPH, 0);
    Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
    XboxController xboxController = new XboxController(0);

    public void Hood(XboxController copilot) {
        this.copilot = copilot;
    }

    @Override
    public void periodic() {
        if (copilot.getAButton()) {
            PH.set(true);
        }
        else if (copilot.getBButton()) {
            PH.set(false);
        }
        else if (!copilot.getBButton() && copilot.getAButton()){
            PH.set(true);
        }
        

    }



}
