package org.team1540.robot2022.commands.climber;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2022.Constants;
import org.team1540.robot2022.Constants.ClimberConstants;
import org.team1540.robot2022.utils.ChickenSmartDashboard;
import org.team1540.robot2022.utils.ChickenTalonFX;

public class Climber extends SubsystemBase {
    private final ChickenTalonFX motorLeft = new ChickenTalonFX(ClimberConstants.Motors.LEFT);
    private final ChickenTalonFX motorRight = new ChickenTalonFX(ClimberConstants.Motors.RIGHT);
    private final ChickenTalonFX[] motors = new ChickenTalonFX[]{motorLeft, motorRight};

    private final DoubleSolenoid solenoid = new DoubleSolenoid(
            Constants.PNEUMATIC_HUB,
            PneumaticsModuleType.REVPH,
            ClimberConstants.Solenoids.SOLENOID_A,
            ClimberConstants.Solenoids.SOLENOID_B
    );

    public Climber() {
        Constants.ShooterConstants.CURRENT_LIMIT_CONFIG.applyTo(new TalonFX[]{motorLeft, motorRight});
        motorLeft.setNeutralMode(NeutralMode.Brake);
        motorRight.setNeutralMode(NeutralMode.Brake);
        motorLeft.setInverted(true);
        motorRight.setInverted(true);

        ChickenSmartDashboard.putDefaultNumber("climber/limits/retracted/leftUp", -645000);
        ChickenSmartDashboard.putDefaultNumber("climber/limits/retracted/rightUp", -645000);
        ChickenSmartDashboard.putDefaultNumber("climber/limits/extended/leftUp", -600000);
        ChickenSmartDashboard.putDefaultNumber("climber/limits/extended/rightUp", -600000);

        updateLimits();
        NetworkTableInstance.getDefault().getTable("SmartDashboard/climber/limits").addEntryListener((table, key, entry, value, flags) -> updateLimits(), EntryListenerFlags.kUpdate);
    }

    /**
     * Factory method zero the encoders
     *
     * @return InstantCommand to zero
     */
    public Command commandZeroEncoders() {
        return new InstantCommand(() -> {
            motorLeft.setSelectedSensorPosition(0);
            motorRight.setSelectedSensorPosition(0);
        }).andThen(this::updateLimits);
    }

    public void periodic() {
        SmartDashboard.putNumber("climber/encoders/left", motorLeft.getSelectedSensorPosition());
        SmartDashboard.putNumber("climber/encoders/right", motorRight.getSelectedSensorPosition());

        SmartDashboard.putNumber("climber/current/left", motorLeft.getStatorCurrent());
        SmartDashboard.putNumber("climber/current/right", motorRight.getStatorCurrent());
    }

    /**
     * Update soft limits
     *
     * @return InstantCommand
     */
    public Command commandUpdateLimits() {
        return new InstantCommand(this::updateLimits);
    }

    /**
     * Disable soft limits for zeroing
     *
     * @return InstantCommand
     */
    public Command commandDisableLimits() {
        return new InstantCommand(() -> {
            motorLeft.configReverseSoftLimitEnable(false);
            motorRight.configReverseSoftLimitEnable(false);
            motorLeft.configForwardSoftLimitEnable(false);
            motorRight.configForwardSoftLimitEnable(false);
        });
    }

    private void updateLimits() {
        double leftUpLimit, rightUpLimit;
        if (this.solenoid.get() == DoubleSolenoid.Value.kForward) { // If retracted, solenoid is inverted
            leftUpLimit = SmartDashboard.getNumber("climber/limits/retracted/leftUp", -470000);
            rightUpLimit = SmartDashboard.getNumber("climber/limits/retracted/rightUp", -470000);
        } else {
            leftUpLimit = SmartDashboard.getNumber("climber/limits/extended/leftUp", -600000);
            rightUpLimit = SmartDashboard.getNumber("climber/limits/extended/rightUp", -600000);
        }
        motorLeft.configReverseSoftLimitEnable(true);
        motorRight.configReverseSoftLimitEnable(true);
        motorLeft.configReverseSoftLimitThreshold(leftUpLimit);
        motorRight.configReverseSoftLimitThreshold(rightUpLimit);

        motorLeft.configForwardSoftLimitEnable(false);
        motorRight.configForwardSoftLimitEnable(false);
        motorLeft.configForwardSoftLimitThreshold(0);
        motorRight.configForwardSoftLimitThreshold(0);
    }

    /**
     * Sets the states of the climber arm solenoids
     *
     * @param forward If the solenoids should be extended forward
     */
    public void setSolenoids(boolean forward) {
        solenoid.set(forward ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
        this.updateLimits();
    }

    /**
     * Set climber output percent power
     *
     * @param left  percentage
     * @param right percentage
     */
    public void setPercent(double left, double right) {
        motorLeft.setPercent(left);
        motorRight.setPercent(right);
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
        motorLeft.setPercent(0);
        motorRight.setPercent(0);
    }

    public double getLeftCurrent() {
        return motorLeft.getStatorCurrent();
    }

    public double getRightCurrent() {
        return motorRight.getStatorCurrent();
    }

    public Command commandSetPercentLeft(double percent) {
        return new InstantCommand(() -> motorLeft.setPercent(percent));
    }

    public Command commandSetPercentRight(double percent) {
        return new InstantCommand(() -> motorRight.setPercent(percent));
    }
}
