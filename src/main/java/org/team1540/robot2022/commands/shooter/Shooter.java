package org.team1540.robot2022.commands.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import org.team1540.robot2022.Constants;
import org.team1540.robot2022.utils.ChickenShuffleboard;
import org.team1540.robot2022.utils.ChickenTalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase {
    public final double rearP = 0.5;
    public final double rearI = 0;
    public final double rearD = 10;
    public final double rearF = 0.048;

    public final double frontP = 0.5;
    public final double frontI = 0;
    public final double frontD = 10;
    public final double frontF = 0.048;

    public ChickenTalonFX shooterMotorFront = new ChickenTalonFX(Constants.ShooterConstants.FRONT);
    public ChickenTalonFX shooterMotorRear = new ChickenTalonFX(Constants.ShooterConstants.REAR);

    public Shooter() {
        Constants.ShooterConstants.CURRENT_LIMIT_CONFIG.applyTo(new ChickenTalonFX[]{shooterMotorFront, shooterMotorRear});
        shooterMotorFront.setNeutralMode(NeutralMode.Coast);
        shooterMotorRear.setNeutralMode(NeutralMode.Coast);
        shooterMotorFront.setInverted(true);
        shooterMotorRear.setInverted(true);
    }

    @Override
    public void periodic() {}

    /**
     * Stop spinning shooter
     */
    public void stop() {
        shooterMotorFront.set(TalonFXControlMode.PercentOutput, 0);
        shooterMotorRear.set(TalonFXControlMode.PercentOutput, 0);
    }

    /**
     * Set motor velocity
     *
     * @param frontVelocity front wheel RPM setpoint
     * @param rearVelocity  front wheel RPM setpoint
     */
    public void setVelocityRPM(double frontVelocity, double rearVelocity) {
        shooterMotorFront.setVelocityRPM(frontVelocity);
        shooterMotorRear.setVelocityRPM(rearVelocity);
    }

    /**
     * Update PID gains from SmartDashboard
     */
    public void updatePIDs() {
        shooterMotorFront.config_kP(0, ChickenShuffleboard.ShooterTab.Tuning.PIDs.frontP.getDouble(frontP));
        shooterMotorFront.config_kI(0, ChickenShuffleboard.ShooterTab.Tuning.PIDs.frontI.getDouble(frontI));
        shooterMotorFront.config_kD(0, ChickenShuffleboard.ShooterTab.Tuning.PIDs.frontD.getDouble(frontD));
        shooterMotorFront.config_kF(0, ChickenShuffleboard.ShooterTab.Tuning.PIDs.frontF.getDouble(frontF));

        shooterMotorRear.config_kP(0, ChickenShuffleboard.ShooterTab.Tuning.PIDs.rearP.getDouble(rearP));
        shooterMotorRear.config_kI(0, ChickenShuffleboard.ShooterTab.Tuning.PIDs.rearI.getDouble(rearI));
        shooterMotorRear.config_kD(0, ChickenShuffleboard.ShooterTab.Tuning.PIDs.rearD.getDouble(rearD));
        shooterMotorRear.config_kF(0, ChickenShuffleboard.ShooterTab.Tuning.PIDs.rearF.getDouble(rearF));
    }

    /**
     * Get front flywheel PID error
     *
     * @return front flywheel PID error
     */
    public double getFrontClosedLoopError() {
        return shooterMotorFront.getClosedLoopError();
    }

    /**
     * Get rear flywheel PID error
     *
     * @return rear flywheel PID error
     */
    public double getRearClosedLoopError() {
        return shooterMotorRear.getClosedLoopError();
    }

    /**
     * Get combined average flywheel PID error
     *
     * @return combined average flywheel PID error
     */
    public double getClosedLoopError() {
        return (Math.abs(getFrontClosedLoopError()) + Math.abs(getRearClosedLoopError())) / 2;
    }

    /**
     * Stop spinning shooter
     *
     * @return new InstantCommand
     */
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
            this.setVelocityRPM(front, rear);
        }, this);
    }

    /**
     * Check if the shooter is spun up close enough to target velocity
     *
     * @return if the shooter is spun up
     */
    public boolean isSpunUp() {
        return getClosedLoopError() < ChickenShuffleboard.ShooterTab.targetError.getDouble(30) &&
               Math.abs(shooterMotorFront.getVelocityRPM() + shooterMotorRear.getVelocityRPM()) > 200; // Make sure the shooter is moving

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
        AUTOMATIC
    }
}
