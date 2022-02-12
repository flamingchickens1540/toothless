package org.team1540.robot2022.commands.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2022.Constants;
import org.team1540.robot2022.LunaUtils.Timer;

public class Shooter extends SubsystemBase {
    private final double kP = 0.5;
    private final double kD = 30;
    private final double kF = 0.0484;
    private final double kI = 0;
    private final double iZone = 0;

    public TalonFX shooterMotorFront = new TalonFX(Constants.ShooterConstants.front);
    public TalonFX shooterMotorRear = new TalonFX(Constants.ShooterConstants.rear);

    private final Timer kdTimer = new Timer();

    public Shooter() {
        setupFlywheelMotors();
        setupPIDs();
        updatePIDs();
        kdTimer.reset();
        kdTimer.start();
    }

    private void setupFlywheelMotors() {
        Constants.ShooterConstants.currentLimitConfig.applyTo(shooterMotorFront);
        Constants.ShooterConstants.currentLimitConfig.applyTo(shooterMotorRear);
        shooterMotorFront.setNeutralMode(NeutralMode.Coast);
        shooterMotorRear.setNeutralMode(NeutralMode.Coast);
        shooterMotorRear.setInverted(TalonFXInvertType.OpposeMaster);
    }

    private void setupPIDs() {
        SmartDashboard.putNumber("shooter/tuning/kP", kP);
        SmartDashboard.putNumber("shooter/tuning/kI", kI);
        SmartDashboard.putNumber("shooter/tuning/iZone", iZone);
        SmartDashboard.putNumber("shooter/tuning/kF", kF);
        SmartDashboard.putNumber("shooter/tuning/kD", kD);

        NetworkTableInstance.getDefault().getTable("SmartDashboard/shooter/tuning").addEntryListener((table, key, entry, value, flags) -> updatePIDs(), EntryListenerFlags.kUpdate);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter/current", shooterMotorFront.getStatorCurrent() + shooterMotorRear.getStatorCurrent());
        SmartDashboard.putNumber("shooter/velocityFront", getFrontVelocityRPM());
        SmartDashboard.putNumber("shooter/velocityRear", getRearVelocityRPM());
        SmartDashboard.putNumber("shooter/error", getClosedLoopError());
        SmartDashboard.putNumber("setFrontVelocityRPM", setFrontVelocityRPM(500));
        SmartDashboard.putNumber("setRearVelocityRPM", setRearVelocityRPM(500));
    }

    public void stop() {
        shooterMotorFront.set(TalonFXControlMode.PercentOutput, 0);
        shooterMotorRear.set(TalonFXControlMode.PercentOutput, 0);
    }

    public double getFrontVelocityRPM() {
        return (shooterMotorFront.getSelectedSensorVelocity() / 2048.0) * 600;
    }

    public double getRearVelocityRPM() {
        return (shooterMotorRear.getSelectedSensorVelocity() / 2048.0) * 600;
    }

    public double setFrontVelocityRPM(double velocity) {
        shooterMotorFront.set(TalonFXControlMode.Velocity, (velocity * 2048.0) / 600);
        return velocity;
    }

    public double setRearVelocityRPM(double velocity) {
        shooterMotorRear.set(TalonFXControlMode.Velocity, (velocity * 2048.0) / 600);
        return velocity;
    }

    private void updatePIDs() {
        shooterMotorFront.config_kP(0, SmartDashboard.getNumber("shooter/tuning/kP", kP));
        shooterMotorFront.config_kI(0, SmartDashboard.getNumber("shooter/tuning/kI", kI));
        shooterMotorFront.config_kD(0, SmartDashboard.getNumber("shooter/tuning/kI", kD));
        shooterMotorFront.config_kF(0, SmartDashboard.getNumber("shooter/tuning/kF", kF));
        shooterMotorFront.config_IntegralZone(0, (int) SmartDashboard.getNumber("shooter/tuning/iZone", iZone));
    }

    public double getClosedLoopError() {
        return shooterMotorFront.getClosedLoopError() + shooterMotorRear.getClosedLoopError();
    }

    public Command commandStop() {
        return new InstantCommand(this::stop, this);
    }
}
