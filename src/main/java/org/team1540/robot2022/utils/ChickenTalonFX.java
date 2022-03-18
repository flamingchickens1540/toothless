package org.team1540.robot2022.utils;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import org.team1540.robot2022.Constants.DriveConstants;

public class ChickenTalonFX extends TalonFX {
    private final double decisecondsPerSecond = 10;
    private double ticksPerMeter = DriveConstants.ENCODER_TICKS_PER_METER;
    private double saturationVoltage = 12;
    private double forwardSoftLimit, reverseSoftLimit;
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
        return this.getSelectedSensorPosition() / ticksPerMeter;
    }

    public double getRateMetersPerSecond() {
        return this.getSelectedSensorVelocity() * decisecondsPerSecond / ticksPerMeter;
    }

    public void setPercent(double percent) {
        this.set(ControlMode.PercentOutput, percent);
    }

    /**
     * Set motor velocity RPM
     * @param rpm motor velocity setpoint
     */
    public void setVelocityRPM(double rpm) {
        this.set(ControlMode.Velocity, (rpm * 2048.0) / 600);
    }

    /**
     * Get motor velocity RPM
     *
     * @return velocity in RPM
     */
    public double getVelocityRPM() {
        return (this.getSelectedSensorVelocity() / 2048.0) * 600;
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


    @Override
    public ErrorCode configForwardSoftLimitThreshold(double forwardSensorLimit) {
        this.forwardSoftLimit = forwardSensorLimit;
        System.out.println(forwardSensorLimit);
        return super.configForwardSoftLimitThreshold(forwardSensorLimit);
    }

    @Override
    public ErrorCode configReverseSoftLimitThreshold(double reverseSoftLimit) {
        this.reverseSoftLimit = reverseSoftLimit;
        System.out.println(reverseSoftLimit);
        return super.configReverseSoftLimitThreshold(reverseSoftLimit);
    }

    /**
     * Sets both the forward and reverse soft limits
     *
     * @param forwardSensorLimit the forward soft limit
     * @param reverseSoftLimit   the reverse soft limit
     */
    public void configSoftLimitThresholds(double forwardSensorLimit, double reverseSoftLimit) {
        this.configForwardSoftLimitThreshold(forwardSensorLimit);
        this.configReverseSoftLimitThreshold(reverseSoftLimit);
    }

    public boolean getForwardSoftLimitReached() {
        return (this.getSelectedSensorPosition() >= this.forwardSoftLimit);
    }

    public boolean getReverseSoftLimitReached() {
        return (this.getSelectedSensorPosition() <= this.reverseSoftLimit);
    }
}
