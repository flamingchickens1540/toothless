package org.team1540.robot2022.commands.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2022.Constants;
import org.team1540.robot2022.Constants.ClimberConstants;
import org.team1540.robot2022.utils.ChickenTalonFX;

public class Climber extends SubsystemBase {
    private final ChickenTalonFX motorLeft = new ChickenTalonFX(ClimberConstants.Motors.left);
    private final ChickenTalonFX motorRight = new ChickenTalonFX(ClimberConstants.Motors.right);

    private final Solenoid solenoidLeft = new Solenoid(Constants.ph, PneumaticsModuleType.REVPH, ClimberConstants.Solenoids.left);
    private final Solenoid solenoidRight = new Solenoid(Constants.ph, PneumaticsModuleType.REVPH, ClimberConstants.Solenoids.right);

    public Climber() {
        Constants.ShooterConstants.currentLimitConfig.applyTo(new TalonFX[]{motorLeft, motorRight});
        motorLeft.setNeutralMode(NeutralMode.Brake);
        motorRight.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Sets the states of the climber arm solenoids
     *
     * @param forward If the solenoids should be extended forward
     */
    public void setSolenoids(boolean forward) {
        // TODO: Make sure this isn't inverted
        solenoidLeft.set(forward);
        solenoidRight.set(forward);
    }

    /**
     * Return the status of both solenoids. Returns true if either solenoid is true.
     * @return AND result of both solenoids
     */
    public boolean getSolenoids() {
        return solenoidLeft.get() && solenoidRight.get();
    }

    public void setLeftPercent(double percent) {
        motorLeft.setPercent(percent);
    }
    public void setRightPercent(double percent) {
        motorRight.setPercent(percent);
    }

    /**
     * Sets the climber arms to a specified position
     *
     * @param position
     */
    public void setMotors(double position) {
        // TODO: Allow specifying length instead of position (and figure out units)
        motorLeft.set(ControlMode.Position, position);
    }

    /**
     * Updates the PIDs on the climber motors from SmartDashboard
     */
    private void updatePIDs() {
        motorLeft.config_kP(0, SmartDashboard.getNumber("climber/PID/kP", 0.3));
    }
}
