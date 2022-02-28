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
    private final ChickenTalonFX motorLeft = new ChickenTalonFX(ClimberConstants.Motors.LEFT);
    private final ChickenTalonFX motorRight = new ChickenTalonFX(ClimberConstants.Motors.RIGHT);
    private final ChickenTalonFX[] motors = new ChickenTalonFX[]{motorLeft, motorRight};

    private final Solenoid solenoidA = new Solenoid(Constants.PNEUMATIC_HUB, PneumaticsModuleType.REVPH, ClimberConstants.Solenoids.SOLENOID_A);
    private final Solenoid solenoidB = new Solenoid(Constants.PNEUMATIC_HUB, PneumaticsModuleType.REVPH, ClimberConstants.Solenoids.SOLENOID_B);

    public Climber() {
        Constants.ShooterConstants.CURRENT_LIMIT_CONFIG.applyTo(new TalonFX[]{motorLeft, motorRight});
        motorLeft.setNeutralMode(NeutralMode.Brake);
        motorRight.setNeutralMode(NeutralMode.Brake);
        motorLeft.setInverted(true);
        motorRight.setInverted(true);
    }

    /**
     * Sets the states of the climber arm solenoids
     *
     * @param forward If the solenoids should be extended forward
     */
    public void setSolenoids(boolean forward) {
        solenoidA.set(forward);
        solenoidB.set(!forward);
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

    /**
     * Sets the NeutralMode for the climber (either coast or brake)
     * @param mode The mode to set the wheels to
     */
    public void setNeutralMode(NeutralMode mode) {
        for (ChickenTalonFX motor : motors) { motor.setNeutralMode(mode); }
    }
}
