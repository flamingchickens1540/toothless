package org.team1540.robot2022.commands.intake;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

//make button controlls commands no control logic in subsystems periodic 

public class Intake extends SubsystemBase {
  Solenoid PH = new Solenoid(PneumaticsModuleType.REVPH, 0); 
  Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
  XboxController xboxController = new XboxController(0); 

  private final XboxController copilot;
  public TalonFX intakeMotorA = new TalonFX(7);

  public Intake(XboxController copilot) {
    this.copilot = copilot;
    //intakeMotorA.configPeakCurrentLimit(10); 
    //intakeMotorA.configPeakCurrentDuration(500); 
  }
  public void disableMotors() {
    intakeMotorA.set(ControlMode.PercentOutput, 0);
}

  @Override
  public void periodic() {
    if (copilot.getBButton()) {
      intakeMotorA.set(ControlMode.PercentOutput, 0.5);
    } else if (copilot.getAButton()) {
      intakeMotorA.set(ControlMode.PercentOutput, -0.3);
    }
    if (!copilot.getBButton() && !copilot.getAButton()) {
      intakeMotorA.set(ControlMode.PercentOutput, 0);
    }

    if(copilot.getRightStickButton())
      {
            PH.set(true);
       }
       if(copilot.getLeftStickButton())
       {
            PH.set(false);
        }
  }

  public void set(double percent) {
    intakeMotorA.set(ControlMode.PercentOutput, percent);
  }

  public void intakeMotors(){
    intakeMotorA.set(ControlMode.PercentOutput, 0.5);
  }
  public void outtakeMotors(){
    intakeMotorA.set(ControlMode.PercentOutput, -0.5); 
  }

  public void intakeUp(){
    if(copilot.getRightStickButton()){
            PH.set(true);
      }
    }

  public void intakeDown(){
    if(copilot.getLeftStickButton())
       {
            PH.set(false);
        }
  }
  
}
