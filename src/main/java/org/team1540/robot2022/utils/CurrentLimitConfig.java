package org.team1540.robot2022.utils;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class CurrentLimitConfig {
    public final double statorLimit, statorThreshCurrent, statorThreshTime;
    public final double supplyLimit, supplyThreshCurrent, supplyThreshTime;

    public CurrentLimitConfig(double statorLimit, double statorThreshCurrent, double statorThreshTime,
                              double supplyLimit, double supplyThreshCurrent, double supplyThreshTime) {
        this.statorLimit = statorLimit;
        this.statorThreshCurrent = statorThreshCurrent;
        this.statorThreshTime = statorThreshTime;

        this.supplyLimit = supplyLimit;
        this.supplyThreshCurrent = supplyThreshCurrent;
        this.supplyThreshTime = supplyThreshTime;
    }

    public void reset(TalonFX motor) {
        TalonFXConfiguration defaultConfig = new TalonFXConfiguration();
        defaultConfig.voltageCompSaturation = 12;
        defaultConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(false, 0, 0, 0);
        defaultConfig.statorCurrLimit = new StatorCurrentLimitConfiguration(false, 0, 0, 0);
        defaultConfig.openloopRamp = 0;

        motor.configFactoryDefault();
        motor.configAllSettings(defaultConfig);
        motor.setNeutralMode(NeutralMode.Coast);
        motor.enableVoltageCompensation(true);
    }

    public void applyTo(TalonFX motor) {
        reset(motor);
        motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, this.statorLimit, this.statorThreshCurrent, this.statorThreshTime));
        motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, this.supplyLimit, this.supplyThreshCurrent, this.supplyThreshTime));
    }
}
