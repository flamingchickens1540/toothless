package org.team1540.robot2022.commands.indexer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2022.Constants.IndexerConstants;
import org.team1540.robot2022.Constants.IndexerConstants.BeamBreaks;
import org.team1540.robot2022.Constants.IndexerConstants.IndexerMotors;
import org.team1540.robot2022.utils.ChickenTalonFX;

public class Indexer extends SubsystemBase {
    public final ChickenTalonFX bottomMotor = new ChickenTalonFX(IndexerMotors.BOTTOM_MOTOR);
    public final ChickenTalonFX topMotor = new ChickenTalonFX(IndexerMotors.TOP_MOTOR);
    private final ChickenTalonFX[] motors = {topMotor, bottomMotor};

    private final DigitalInput topSensor = new DigitalInput(BeamBreaks.TOP_INDEXER_SENSOR);
    private final DigitalInput bottomSensor = new DigitalInput(BeamBreaks.BOTTOM_INDEXER_SENSOR);

    // If standby is false, the indexer will stop when it has both balls
    private boolean standby = false;

    private final AsynchronousInterrupt topInterrupt = new AsynchronousInterrupt(topSensor, (rising, falling) -> {
        // These rising/falling booleans are both reporting false, and I don't know why
        if (getTopSensor() && !standby) { // Stop top indexer if ball is there and not in standby
            set(IndexerState.OFF, IndexerState.UNCHANGED);
        }
    });

    private final AsynchronousInterrupt bottomInterrupt = new AsynchronousInterrupt(bottomSensor, (rising, falling) -> {
        // These rising/falling booleans are both reporting false, and I don't know why
        if (isFull() && !standby) { // Stop bottom indexer if indexer is full and not in standby
            set(IndexerState.UNCHANGED, IndexerState.OFF);
        }
    });

    public Indexer(NeutralMode brakeType) {
        topInterrupt.setInterruptEdges(true, true);
        bottomInterrupt.setInterruptEdges(true, true);
        topInterrupt.enable();
        bottomInterrupt.enable();

        IndexerMotors.CURRENT_LIMIT_CONFIG.applyTo(motors);
        for (ChickenTalonFX motor : motors) {
            motor.setNeutralMode(brakeType);
            motor.setInverted(true);
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("indexer/top", this.getTopSensor());
        SmartDashboard.putBoolean("indexer/bottom", this.getBottomSensor());
        SmartDashboard.putBoolean("indexer/full", this.isFull());
    }

    /**
     * Returns true if the top beam break is blocked
     *
     * @return if the sensor is blocked
     */
    public boolean getTopSensor() {
        return !topSensor.get();
    }

    /**
     * Returns true if the bottom beam break is blocked
     *
     * @return if the sensor is blocked
     */
    public boolean getBottomSensor() {
        return !bottomSensor.get();
    }

    /**
     * Returns true if the top and bottom sensors are blocked
     *
     * @return if both sensors are blocked
     */
    public boolean isFull() {
        return (this.getBottomSensor() && this.getTopSensor());
    }

    /**
     * Controls indexer standby mode. If standby is disabled, the indexer will stop when it has both balls
     * @param standby indexer standby state
     */
    public void setStandby(boolean standby) {
        this.standby = standby;
    }

    /**
     * Sets the top and bottom indexer motors to an IndexerMode
     *
     * @param topMode    The mode for the top motor
     * @param bottomMode The mode for the bottom motor
     */
    public void set(IndexerState topMode, IndexerState bottomMode) {
        this.setTop(topMode);
        this.setBottom(bottomMode);
    }

    /**
     * Sets the top indexer motor to an IndexerMode
     *
     * @param mode The mode to set the motor to
     */
    public void setTop(IndexerState mode) {
        this.setMotor(topMotor, mode, IndexerConstants.TOP_PERCENT);
    }

    /**
     * Sets the bottom indexer motor to an IndexerMode
     *
     * @param mode The mode to set the motor to
     */
    public void setBottom(IndexerState mode) {
        this.setMotor(bottomMotor, mode, IndexerConstants.BOTTOM_PERCENT);
    }

    /**
     * Sets a motor to an IndexerState
     *
     * @param motor     The motor to run
     * @param mode      The mode to set it to
     * @param onPercent the speed to use for running forward and reverse
     */
    private void setMotor(ChickenTalonFX motor, IndexerState mode, double onPercent) {
        switch (mode) {
            case FORWARD_FULL:
                motor.set(ControlMode.PercentOutput, 1);
                break;
            case FORWARD:
                motor.set(ControlMode.PercentOutput, onPercent);
                break;
            case REVERSE:
                motor.set(ControlMode.PercentOutput, -onPercent);
                break;
            case OFF:
                motor.set(ControlMode.PercentOutput, 0);
                break;
            case UNCHANGED:
                break;
        }
    }

    /**
     * Returns a command that sets the top and bottom indexer motors to an
     * IndexerMode
     *
     * @param topMode    The mode for the top motor
     * @param bottomMode The mode for the bottom motor
     * @return an InstantCommand that sets the motors
     */
    public Command commandSet(IndexerState topMode, IndexerState bottomMode) {
        return new InstantCommand(() -> {
            this.set(topMode, bottomMode);
        });
    }

    /**
     * Returns a command that sets the top indexer motor to an IndexerMode
     *
     * @param mode The mode for the top motor
     * @return an InstantCommand that sets the motor
     */
    public Command commandSetTop(IndexerState mode) {
        return new InstantCommand(() -> {
            this.setTop(mode);
        });
    }

    /**
     * Returns a command that sets the bottom indexer motor to an IndexerMode
     *
     * @param mode The mode for the bottom motor
     * @return an InstantCommand that sets the motor
     */
    public Command commandSetBottom(IndexerState mode) {
        return new InstantCommand(() -> {
            this.setBottom(mode);
        });
    }

    /**
     * Stops both indexer motors
     */
    public void stop() {
        this.set(IndexerState.OFF, IndexerState.OFF);
        this.standby = true;
    }

    /**
     * Returns a command to stop both indexer motors
     *
     * @return an InstantCommand to stop the indexer motors
     */
    public Command commandStop() {
        return new InstantCommand(this::stop);
    }

    /**
     * Sets the NeutralMode for the drivetrain (either coast or brake)
     *
     * @param mode The mode to set the wheels to
     */
    public void setNeutralMode(NeutralMode mode) {
        for (ChickenTalonFX motor : motors) {
            motor.setNeutralMode(mode);
        }
    }

    /**
     * States to set an indexer motor to
     */
    public enum IndexerState {
        /**
         * Full speed forward to shoot
         */
        FORWARD_FULL,
        /**
         * Runs the motor forward
         */
        FORWARD,
        /**
         * Runs the motor in reverse
         */
        REVERSE,
        /**
         * Stops the motor
         */
        OFF,
        /**
         * Does not modify the motor's state
         */
        UNCHANGED
    }
}
