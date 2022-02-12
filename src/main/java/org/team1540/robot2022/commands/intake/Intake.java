package org.team1540.robot2022.commands.intake;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import org.team1540.robot2022.Constants;

public class Intake extends SubsystemBase {
    private final Solenoid leftSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.IntakeConstants.leftSolenoidChannel);
    private final Solenoid rightSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.IntakeConstants.rightSolenoidChannel);
    private final TalonFX motor = new TalonFX(Constants.IntakeConstants.falcon);

    public Intake() {
        motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(Constants.IntakeConstants.statorCurrentLimitEnable, Constants.IntakeConstants.statorCurrentLimit, Constants.IntakeConstants.statorCurrentThreshCurrent, Constants.IntakeConstants.statorCurrentThreshTime));
        motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(Constants.IntakeConstants.supplyCurrentLimitEnable, Constants.IntakeConstants.supplyCurrentLimit, Constants.IntakeConstants.supplyCurrentThreshCurrent, Constants.IntakeConstants.supplyCurrentThreshTime));
    }

    @Override
    public void periodic() {
    }

    public void setTilt(Boolean state) {
        leftSolenoid.set(state);
        rightSolenoid.set(state);
    }

    public void setPercent(double percent) {
        motor.set(ControlMode.PercentOutput, percent);
    }
}
