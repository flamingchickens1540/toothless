package org.team1540.robot2022.commands.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2022.Constants;
import org.team1540.robot2022.utils.ChickenTalonFX;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;

public class Shooter extends SubsystemBase {
    private final double rearP = 0.5;
    private final double rearI = 0;
    private final double rearD = 10;
    private final double rearF = 0.048;

    private final double frontP = 0.5;
    private final double frontI = 0;
    private final double frontD = 10;
    private final double frontF = 0.048;

    public ChickenTalonFX shooterMotorFront = new ChickenTalonFX(Constants.ShooterConstants.FRONT);
    public ChickenTalonFX shooterMotorRear = new ChickenTalonFX(Constants.ShooterConstants.REAR);

    public Shooter() {
        Constants.ShooterConstants.CURRENT_LIMIT_CONFIG.applyTo(new ChickenTalonFX[]{shooterMotorFront, shooterMotorRear});
        shooterMotorFront.setNeutralMode(NeutralMode.Coast);
        shooterMotorRear.setNeutralMode(NeutralMode.Coast);
        shooterMotorFront.setInverted(true);
        shooterMotorRear.setInverted(true);

        SmartDashboard.putNumber("shooter/tuning/frontP", frontP);
        SmartDashboard.putNumber("shooter/tuning/frontI", frontI);
        SmartDashboard.putNumber("shooter/tuning/frontF", frontF);
        SmartDashboard.putNumber("shooter/tuning/frontD", frontD);
        SmartDashboard.putNumber("shooter/tuning/rearP", rearP);
        SmartDashboard.putNumber("shooter/tuning/rearI", rearI);
        SmartDashboard.putNumber("shooter/tuning/rearF", rearF);
        SmartDashboard.putNumber("shooter/tuning/rearD", rearD);
        NetworkTableInstance.getDefault().getTable("SmartDashboard/shooter/tuning").addEntryListener((table, key, entry, value, flags) -> updatePIDs(), EntryListenerFlags.kUpdate);

        SmartDashboard.putNumber("shooter/tuning/waitAfterFirstBall", 0.5);

        SmartDashboard.putNumber("shooter/lastShot/frontRPM", 0);
        SmartDashboard.putNumber("shooter/lastShot/rearRPM", 0);
        SmartDashboard.putNumber("shooter/lastShot/distanceFromTarget", 0);
        SmartDashboard.putBoolean("shooter/lastShot/hoodState", false);

        updatePIDs();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("shooter/current", shooterMotorFront.getStatorCurrent() + shooterMotorRear.getStatorCurrent());
        SmartDashboard.putNumber("shooter/velocityFront", getVelocityRPM(shooterMotorFront));
        SmartDashboard.putNumber("shooter/velocityRear", getVelocityRPM(shooterMotorRear));
        SmartDashboard.putNumber("shooter/error", getClosedLoopError());
        SmartDashboard.putNumber("shooter/error/front", getFrontClosedLoopError());
        SmartDashboard.putNumber("shooter/error/rear", getRearClosedLoopError());
        SmartDashboard.putBoolean("shooter/isSpunUp", isSpunUp());
    }

    public void stop() {
        shooterMotorFront.set(TalonFXControlMode.PercentOutput, 0);
        shooterMotorRear.set(TalonFXControlMode.PercentOutput, 0);
    }

    /**
     * Get motor velocity RPM
     *
     * @param motor to query
     * @return velocity in RPM
     */
    public double getVelocityRPM(TalonFX motor) {
        return (motor.getSelectedSensorVelocity() / 2048.0) * 600;
    }

    /**
     * Set motor velocity
     *
     * @param motor    to set
     * @param velocity to set in RPM
     */
    public void setVelocityRPM(TalonFX motor, double velocity) {
        if (motor == shooterMotorFront) {
            SmartDashboard.putNumber("shooter/frontVelocitySetpoint", velocity);
        } else if (motor == shooterMotorRear) {
            SmartDashboard.putNumber("shooter/rearVelocitySetpoint", velocity);
        }
        motor.set(TalonFXControlMode.Velocity, (velocity * 2048.0) / 600);
    }

        /**
     * Set motor velocity
     *
     * @param motor    to set
     * @param velocity to set in RPM
     */
    public void setVelocityRPM(double frontVelocity, double rearVelocity) {
        shooterMotorFront.set(TalonFXControlMode.Velocity, (frontVelocity * 2048.0) / 600);
        shooterMotorRear.set(TalonFXControlMode.Velocity, (rearVelocity* 2048.0) / 600);
    }

    public void updatePIDs() {
        shooterMotorFront.config_kP(0, SmartDashboard.getNumber("shooter/tuning/frontP", frontP));
        shooterMotorFront.config_kI(0, SmartDashboard.getNumber("shooter/tuning/frontI", frontI));
        shooterMotorFront.config_kD(0, SmartDashboard.getNumber("shooter/tuning/frontD", frontD));
        shooterMotorFront.config_kF(0, SmartDashboard.getNumber("shooter/tuning/frontF", frontF));

        shooterMotorRear.config_kP(0, SmartDashboard.getNumber("shooter/tuning/rearP", rearP));
        shooterMotorRear.config_kI(0, SmartDashboard.getNumber("shooter/tuning/rearI", rearI));
        shooterMotorRear.config_kD(0, SmartDashboard.getNumber("shooter/tuning/rearD", rearD));
        shooterMotorRear.config_kF(0, SmartDashboard.getNumber("shooter/tuning/rearF", rearF));
    }

    public double getFrontClosedLoopError() {
        return shooterMotorFront.getClosedLoopError();
    }

    public double getRearClosedLoopError() {
        return shooterMotorRear.getClosedLoopError();
    }

    public double getClosedLoopError() {
        return (Math.abs(getFrontClosedLoopError()) + Math.abs(getRearClosedLoopError()))/2;
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
            System.out.println("!!!!!!!!!"+front+" - "+rear);
            this.setVelocityRPM(front, rear);
        }, this);
    }

    /**
     * Check if the shooter is spun up close enough to target velocity
     *
     * @return if the shooter is spun up
     */
    public boolean isSpunUp() {
        return getClosedLoopError() < SmartDashboard.getNumber("shooter/tuning/targetError", 30)
                && Math.abs(getVelocityRPM(shooterMotorFront) + getVelocityRPM(shooterMotorRear)) > 200; // Make sure the shooter is moving
    }

    public enum ShooterProfile {
        /**
         * Touching the hub, low goal
         */
        LOWGOAL,

        /**
         * Touching the hub, high goal
         */
        HUB,

        /**
         * Within the tarmac
         */
        TARMAC,

        /**
         * Behind the tarmac
         */
        FAR,

        /**
         * Choose HUB, TARMAC, or FAR based on LIDAR distance
         */
        AUTOMATIC,
    }
}
