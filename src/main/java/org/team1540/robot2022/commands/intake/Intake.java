package org.team1540.robot2022.commands.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2022.Constants;

public class Intake extends SubsystemBase {
    private final Solenoid solenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.IntakeConstants.solenoid);
    private final TalonFX motor = new TalonFX(Constants.IntakeConstants.falcon);

    public Intake() {
        Constants.IntakeConstants.currentLimitConfig.applyTo(motor);
    }

    @Override
    public void periodic() {
    }

    public void setFold(boolean state) {
        // TODO: Make sure the solenoid is configured so that setFold(true) moves the intake to the upwards (folded) position
        solenoid.set(state);
    }

    public void setPercent(double percent) {
        motor.set(ControlMode.PercentOutput, percent);
    }
}
