package org.team1540.robot2022.commands.climber;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2022.Constants;
import org.team1540.robot2022.Constants.ClimberConstants;
import org.team1540.robot2022.utils.ChickenSmartDashboard;
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

    public final DigitalInput sensorLeft = new DigitalInput(3);
    public final DigitalInput sensorRight = new DigitalInput(2);
    private boolean limitsEnabled = true;

    private final AsynchronousInterrupt leftInterrupt = new AsynchronousInterrupt(sensorLeft, (rising, falling) -> {
        // These rising/falling booleans are both reporting false, and I don't know why
        if (!sensorLeft.get()) {
            motorLeft.configReverseSoftLimitThreshold(motorLeft.getSelectedSensorPosition());
            motorLeft.configReverseSoftLimitEnable(true);
        } else {
            motorLeft.configReverseSoftLimitEnable(false);
        }
    });

    private final AsynchronousInterrupt rightInterrupt = new AsynchronousInterrupt(sensorRight, (rising, falling) -> {
        boolean sensorVal = sensorRight.get();
        if (!sensorVal) {
            motorRight.configReverseSoftLimitThreshold(motorRight.getSelectedSensorPosition());
            motorRight.configReverseSoftLimitEnable(true);
        } else {
            motorRight.configReverseSoftLimitEnable(false);
        }
    });


    public Climber() {

        leftInterrupt.setInterruptEdges(true, true);
        rightInterrupt.setInterruptEdges(true, true);
        enableLimits();


        motorLeft.configFactoryDefault();
        motorRight.configFactoryDefault();
        Constants.ShooterConstants.CURRENT_LIMIT_CONFIG.applyTo(motorLeft, motorRight);
        motorLeft.setNeutralMode(NeutralMode.Brake);
        motorRight.setNeutralMode(NeutralMode.Brake);
        motorLeft.setInverted(true);
        motorRight.setInverted(true);
    }

    public void periodic() {
        ChickenSmartDashboard.putDebugNumber("climber/encoders/left", motorLeft.getSelectedSensorPosition());
        ChickenSmartDashboard.putDebugNumber("climber/encoders/right", motorRight.getSelectedSensorPosition());

        ChickenSmartDashboard.putDebugNumber("climber/current/left", motorLeft.getStatorCurrent());
        ChickenSmartDashboard.putDebugNumber("climber/current/right", motorRight.getStatorCurrent());


    }

    public Command commandDisableLimits() {
        return new InstantCommand(() -> {
            System.out.println("disabling climber limits");
            leftInterrupt.disable();
            rightInterrupt.disable();
            motorLeft.configReverseSoftLimitEnable(false);
            motorRight.configReverseSoftLimitEnable(false);
        });
    }

    public void enableLimits() {
        leftInterrupt.enable();
        rightInterrupt.enable();
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

    /**
     * Returns if the arms can be moved
     *
     * @param sensor  The magnet sensor to use
     * @param percent The percent the arms will be moved (to check if it is going down_
     * @return If the sensor is not tripped, the arms will be moved down, or the limits are disabled
     */
    private boolean inThreshold(DigitalInput sensor, double percent) {
        return sensor.get() || percent > 0 || !limitsEnabled;
    }

    public void setPercentLeft(double percent) {
        motorLeft.setPercent(percent);
    }

    public void setPercentRight(double percent) {
        motorRight.setPercent(percent);
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
