package org.team1540.robot2022.commands.indexer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import org.team1540.robot2022.Constants.IndexerConstants;
import org.team1540.robot2022.Constants.IndexerConstants.BeamBreaks;
import org.team1540.robot2022.Constants.IndexerConstants.IndexerMotors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BiConsumer;

public class Indexer extends SubsystemBase {
    private final TalonFX bottomMotor = new TalonFX(IndexerMotors.bottomMotor);
    private final TalonFX topMotor = new TalonFX(IndexerMotors.topMotor);
    private final TalonFX[] motors = {topMotor, bottomMotor};

    private final DigitalInput topSensor = new DigitalInput(BeamBreaks.topIndexerSensor);
    private final DigitalInput bottomSensor = new DigitalInput(BeamBreaks.bottomIndexerSensor);

    private final AsynchronousInterrupt topInterrupt = new AsynchronousInterrupt(topSensor, (rising, falling) -> {
        // These rising/falling booleans are both reporting false, and I don't know why
        // System.out.println("Top sensor: " + rising + " falling " + falling);

        if (getTopSensor()) { // Stop top indexer if ball is there
            set(IndexerState.OFF, IndexerState.UNCHANGED);
        } else {
            set(IndexerState.FORWARD, IndexerState.OFF);
        }
    });

    private final AsynchronousInterrupt bottomInterrupt = new AsynchronousInterrupt(bottomSensor, (rising, falling) -> {
        // These rising/falling booleans are both reporting false, and I don't know why
        // System.out.println("Bottom sensor: " + rising + " falling " + falling);

        if (getBottomSensor()) { // Stop bottom indexer if ball is there
            set(IndexerState.UNCHANGED, IndexerState.OFF);
        } else {
            set(IndexerState.OFF, IndexerState.FORWARD);
        }
    });

    public Indexer(NeutralMode brakeType) {
        topInterrupt.setInterruptEdges(true, true);
        bottomInterrupt.setInterruptEdges(true, true);
        topInterrupt.enable();
        bottomInterrupt.enable();

        IndexerMotors.currentLimitConfig.applyTo(motors);
        for (TalonFX motor : motors) {
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

    public boolean isFull() {
        return (this.getBottomSensor() && this.getTopSensor());
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
        this.setMotor(topMotor, mode, IndexerConstants.topPercent);
    }

    /**
     * Sets the bottom indexer motor to an IndexerMode
     *
     * @param mode The mode to set the motor to
     */
    public void setBottom(IndexerState mode) {
        this.setMotor(bottomMotor, mode, IndexerConstants.bottomPercent);
    }

    /**
     * Sets a motor to an IndexerState
     *
     * @param motor     The motor to run
     * @param mode      The mode to set it to
     * @param onPercent the speed to use for running forward and reverse
     */
    private void setMotor(TalonFX motor, IndexerState mode, double onPercent) {
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
