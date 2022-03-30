package org.team1540.robot2022.commands.climber;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2022.Constants;
import org.team1540.robot2022.Constants.ClimberConstants;
import org.team1540.robot2022.utils.ChickenTalonFX;

public class Climber extends SubsystemBase {
    public final ChickenTalonFX motorLeft = new ChickenTalonFX(ClimberConstants.Motors.LEFT);
    public final ChickenTalonFX motorRight = new ChickenTalonFX(ClimberConstants.Motors.RIGHT);
    private final ChickenTalonFX[] motors = new ChickenTalonFX[]{motorLeft, motorRight};

    private final DoubleSolenoid solenoid = new DoubleSolenoid(
            Constants.PNEUMATIC_HUB,
            PneumaticsModuleType.REVPH,
            ClimberConstants.Solenoids.SOLENOID_A,
            ClimberConstants.Solenoids.SOLENOID_B
    );

    public final DigitalInput sensorLeft = new DigitalInput(2);
    public final DigitalInput sensorRight = new DigitalInput(3);
    private boolean limitsEnabled = true;


    public Climber() {
        Constants.ShooterConstants.CURRENT_LIMIT_CONFIG.applyTo(new TalonFX[]{motorLeft, motorRight});
        motorLeft.setNeutralMode(NeutralMode.Brake);
        motorRight.setNeutralMode(NeutralMode.Brake);
        motorLeft.setInverted(true);
        motorRight.setInverted(true);
    }

    public void periodic() {
        SmartDashboard.putNumber("climber/encoders/left", motorLeft.getSelectedSensorPosition());
        SmartDashboard.putNumber("climber/encoders/right", motorRight.getSelectedSensorPosition());

        SmartDashboard.putNumber("climber/current/left", motorLeft.getStatorCurrent());
        SmartDashboard.putNumber("climber/current/right", motorRight.getStatorCurrent());


    }

    public Command commandDisableLimits() {
        return new InstantCommand(
                () -> this.limitsEnabled = false
        );
    }


    /**
     * Sets the states of the climber arm solenoids
     *
     * @param forward If the solenoids should be extended forward
     */
    public void setSolenoids(boolean forward) {
        solenoid.set(forward ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }

    /**
     * Set climber output percent power (positive values lower the arms)
     *
     * @param left  percentage
     * @param right percentage
     */
    public void setPercent(double left, double right) {
        setPercentLeft(left);
        setPercentRight(right);
    }


    /**
     * Sets the NeutralMode for the climber (either coast or brake)
     *
     * @param mode The mode to set the wheels to
     */
    public void setNeutralMode(NeutralMode mode) {
        for (ChickenTalonFX motor : motors) {
            motor.setNeutralMode(mode);
        }
    }

    /**
     * Stop climber motors
     */
    public void stop() {
        setPercentLeft(0);
        setPercentRight(0);
    }

    public void setPercentLeft(double percent) {
        System.out.println(percent);
        if (this.sensorLeft.get() || percent > 0) {
            motorLeft.setPercent(percent);
        } else {
            motorLeft.setPercent(0);
        }
    }

    public void setPercentRight(double percent) {

        if (this.sensorRight.get() || percent > 0) {
            motorRight.setPercent(percent);
        } else {
            motorRight.setPercent(0);
        }
    }

    public double getLeftCurrent() {
        return motorLeft.getStatorCurrent();
    }

    public double getRightCurrent() {
        return motorRight.getStatorCurrent();
    }

    public Command commandSetPercentLeft(double percent) {
        return new InstantCommand(() -> setPercentLeft(percent));
    }

    public Command commandSetPercentRight(double percent) {
        return new InstantCommand(() -> setPercentRight(percent));
    }

    public Command commandSetPercent(double percent) {
        return new InstantCommand(
                () -> {
                    setPercentRight(percent);
                    setPercentLeft(percent);
                }
        );
    }
}
