package org.team1540.robot2022.commands.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2022.Constants;

public class Shooter extends SubsystemBase {
    private final double rearP = 0.5;
    private final double rearD = 30;
    private final double rearF = 0.0484;
    private final double rearI = 0;
    private final double frontP = 0.5;
    private final double frontD = 30;
    private final double frontF = 0.0484;
    private final double frontI = 0;

    public TalonFX shooterMotorFront = new TalonFX(Constants.ShooterConstants.front);
    public TalonFX shooterMotorRear = new TalonFX(Constants.ShooterConstants.rear);

    public Shooter() {
        Constants.ShooterConstants.currentLimitConfig.applyTo(new TalonFX[]{shooterMotorFront, shooterMotorRear});
        shooterMotorFront.setNeutralMode(NeutralMode.Coast);
        shooterMotorRear.setNeutralMode(NeutralMode.Coast);

        SmartDashboard.putNumber("shooter/tuning/frontP", frontP);
        SmartDashboard.putNumber("shooter/tuning/frontI", frontI);
        SmartDashboard.putNumber("shooter/tuning/frontF", frontF);
        SmartDashboard.putNumber("shooter/tuning/frontD", frontD);
        SmartDashboard.putNumber("shooter/tuning/rearP", rearP);
        SmartDashboard.putNumber("shooter/tuning/rearI", rearI);
        SmartDashboard.putNumber("shooter/tuning/rearF", rearF);
        SmartDashboard.putNumber("shooter/tuning/rearD", rearD);
        NetworkTableInstance.getDefault().getTable("SmartDashboard/shooter/tuning").addEntryListener((table, key, entry, value, flags) -> updatePIDs(), EntryListenerFlags.kUpdate);

        updatePIDs();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter/current", shooterMotorFront.getStatorCurrent() + shooterMotorRear.getStatorCurrent());
        SmartDashboard.putNumber("shooter/velocityFront", getVelocityRPM(shooterMotorFront));
        SmartDashboard.putNumber("shooter/velocityRear", getVelocityRPM(shooterMotorRear));
        SmartDashboard.putNumber("shooter/error", getClosedLoopError());
    }

    public void stop() {
        shooterMotorFront.set(TalonFXControlMode.PercentOutput, 0);
        shooterMotorRear.set(TalonFXControlMode.PercentOutput, 0);
    }

    /**
     * Get motor velocity RPM
     * @param motor to query
     * @return velocity in RPM
     */
    public double getVelocityRPM(TalonFX motor) {
        return (motor.getSelectedSensorVelocity() / 2048.0) * 600;
    }

    /**
     * Set motor velocity
     * @param motor to set
     * @param velocity to set in RPM
     */
    public void setVelocityRPM(TalonFX motor, double velocity) {
        motor.set(TalonFXControlMode.Velocity, (velocity * 2048.0) / 600);
    }

    private void updatePIDs() {
        shooterMotorFront.config_kP(0, SmartDashboard.getNumber("shooter/tuning/frontP", frontP));
        shooterMotorFront.config_kI(0, SmartDashboard.getNumber("shooter/tuning/frontI", frontI));
        shooterMotorFront.config_kD(0, SmartDashboard.getNumber("shooter/tuning/frontI", frontD));
        shooterMotorFront.config_kF(0, SmartDashboard.getNumber("shooter/tuning/frontF", frontF));

        shooterMotorRear.config_kP(0, SmartDashboard.getNumber("shooter/tuning/rearP", rearP));
        shooterMotorRear.config_kI(0, SmartDashboard.getNumber("shooter/tuning/rearI", rearI));
        shooterMotorRear.config_kD(0, SmartDashboard.getNumber("shooter/tuning/rearI", rearD));
        shooterMotorRear.config_kF(0, SmartDashboard.getNumber("shooter/tuning/rearF", rearF));
    }

    public double getClosedLoopError() {
        return shooterMotorFront.getClosedLoopError() + shooterMotorRear.getClosedLoopError();
    }

    public Command commandStop() {
        return new InstantCommand(this::stop, this);
    }

    /**
     * Factory method to set shooter velocity
     *
     * @param front velocity RPM
     * @param rear  velocity RPM
     * @return new InstantCommand
     */
    public Command commandSetVelocity(double front, double rear) {
        return new InstantCommand(() -> {
            shooterMotorFront.set(TalonFXControlMode.Velocity, front);
            shooterMotorRear.set(TalonFXControlMode.Velocity, rear);
        }, this);
    }
}
