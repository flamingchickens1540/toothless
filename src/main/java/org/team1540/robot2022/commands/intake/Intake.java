package org.team1540.robot2022.commands.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2022.Constants;

public class Intake extends SubsystemBase {
    private final Solenoid solenoid = new Solenoid(Constants.ph, PneumaticsModuleType.REVPH, Constants.IntakeConstants.solenoid);
    private final TalonFX motor = new TalonFX(Constants.IntakeConstants.falcon);

    public Intake() {
        Constants.IntakeConstants.currentLimitConfig.applyTo(motor);
    }

    @Override
    public void periodic() {
    }

    /**
     * Checks if the intake is retracted
     * @return whether the intake is retracted
     */
    public boolean getFold() {
        return !solenoid.get();
    }

    /**
     * Retracts or lowers the intake
     * @param isUp whether the intake should be retracted
     */
    public void setFold(boolean isUp) {
        solenoid.set(!isUp);
    }

    public void setPercent(double percent) {
        motor.set(ControlMode.PercentOutput, percent);
    }

    public void stop() {
        this.setPercent(0);
    }

    public Command commandStop() {
        return new InstantCommand(this::stop);
    }

    /**
     * Returns a command to set the fold state of the indexer
     * @param isUp if the indexer is folded up
     * @return An InstantCommand
     */
    public Command commandSetFold(boolean isUp) {
        return new InstantCommand(() -> this.setFold(isUp));
    }
}
