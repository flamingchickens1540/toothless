package org.team1540.robot2022.commands.Intake;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

//make button controlls commands no control logic in subsystems periodic 

public class Intake extends SubsystemBase {
  Solenoid PH = new Solenoid(PneumaticsModuleType.REVPH, 0); 
  Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);
  XboxController xboxController = new XboxController(0); 

  private final XboxController copilot;
  public TalonSRX shooterMotorA = new TalonSRX(7);

  public Intake(XboxController copilot) {
    this.copilot = copilot;
    shooterMotorA.configPeakCurrentLimit(10); 
    shooterMotorA.configPeakCurrentDuration(500); 
  }
  public void disableMotors() {
    shooterMotorA.set(ControlMode.PercentOutput, 0);
}

  @Override
  public void periodic() {
    if (copilot.getBButton()) {
      shooterMotorA.set(ControlMode.PercentOutput, 0.5);
    } else if (copilot.getAButton()) {
      shooterMotorA.set(ControlMode.PercentOutput, -0.3);
    }
    if (!copilot.getBButton() && !copilot.getAButton()) {
      shooterMotorA.set(ControlMode.PercentOutput, 0);
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
    shooterMotorA.set(ControlMode.PercentOutput, percent);
  }

  public void intakeMotors(){
    shooterMotorA.set(ControlMode.PercentOutput, 0.5);
  }
  public void outtakeMotors(){
    shooterMotorA.set(ControlMode.PercentOutput, -0.5); 
  }
  
}
