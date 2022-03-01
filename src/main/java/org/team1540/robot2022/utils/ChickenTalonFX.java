package org.team1540.robot2022.utils;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.team1540.robot2022.Constants.DriveConstants;

public class ChickenTalonFX extends TalonFX {
    private final double decisecondsPerSecond = 10;
    private double ticksPerMeter = DriveConstants.ENCODER_TICKS_PER_METER;
    private double saturationVoltage = 12;
    private int velocitySlotIdx = 0;
    private int positionSlotIdx = 1;
    private int lastSlot = 0;

    /**
     * Constructor
     *
     * @param deviceNumber [0,62]
     */
    public ChickenTalonFX(int deviceNumber) {
        super(deviceNumber);
    }

    public double getDistanceMeters() {
        return (double) this.getSelectedSensorPosition() / ticksPerMeter;
    }

    public double getRateMetersPerSecond() {
        return (double) this.getSelectedSensorVelocity() * decisecondsPerSecond / ticksPerMeter;
    }

    public void setPercent(double percent) {
        this.set(ControlMode.PercentOutput, percent);
    }

    public void setVelocityMetersPerSecond(double velocityMetersPerSecond) {
        if (lastSlot != velocitySlotIdx)
            selectProfileSlot(velocitySlotIdx, 0);
        this.set(ControlMode.Velocity, velocityMetersPerSecond * ticksPerMeter / decisecondsPerSecond);
    }

    public void setPositionMeters(double positionMeters) {
        if (lastSlot != positionSlotIdx)
            selectProfileSlot(positionSlotIdx, 0);
        this.set(ControlMode.Position, positionMeters * ticksPerMeter);
    }

    public void setVoltage(double voltage) {
        this.set(ControlMode.PercentOutput, voltage / saturationVoltage);
    }

    @Override
    public void selectProfileSlot(int slotIdx, int pidIdx) {
        lastSlot = slotIdx;
        super.selectProfileSlot(slotIdx, pidIdx);
    }

    @Override
    public ErrorCode configVoltageCompSaturation(double voltage, int timeoutMs) {
        saturationVoltage = voltage;
        return super.configVoltageCompSaturation(voltage, timeoutMs);
    }

    @Override
    public ErrorCode configVoltageCompSaturation(double voltage) {
        saturationVoltage = voltage;
        return super.configVoltageCompSaturation(voltage);
    }
}
