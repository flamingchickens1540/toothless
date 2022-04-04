package org.team1540.robot2022.commands.shooter;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2022.Constants;
import org.team1540.robot2022.utils.ChickenSmartDashboard;
import org.team1540.robot2022.utils.ChickenTalonFX;

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
//        Constants.ShooterConstants.CURRENT_LIMIT_CONFIG.applyTo(new ChickenTalonFX[]{shooterMotorFront, shooterMotorRear});
        shooterMotorFront.configFactoryDefault();
        shooterMotorRear.configFactoryDefault();
        shooterMotorFront.setNeutralMode(NeutralMode.Coast);
        shooterMotorRear.setNeutralMode(NeutralMode.Coast);
        shooterMotorFront.setInverted(true);
        shooterMotorRear.setInverted(true);

        ChickenSmartDashboard.putDefaultNumber("shooter/tuning/frontP", frontP);
        ChickenSmartDashboard.putDefaultNumber("shooter/tuning/frontI", frontI);
        ChickenSmartDashboard.putDefaultNumber("shooter/tuning/frontF", frontF);
        ChickenSmartDashboard.putDefaultNumber("shooter/tuning/frontD", frontD);
        ChickenSmartDashboard.putDefaultNumber("shooter/tuning/rearP", rearP);
        ChickenSmartDashboard.putDefaultNumber("shooter/tuning/rearI", rearI);
        ChickenSmartDashboard.putDefaultNumber("shooter/tuning/rearF", rearF);
        ChickenSmartDashboard.putDefaultNumber("shooter/tuning/rearD", rearD);
        NetworkTableInstance.getDefault().getTable("SmartDashboard/shooter/tuning").addEntryListener((table, key, entry, value, flags) -> updatePIDs(), EntryListenerFlags.kUpdate);

        ChickenSmartDashboard.putDefaultNumber("shooter/tuning/waitAfterFirstBall", 0.5);

        ChickenSmartDashboard.putDebugNumber("shooter/lastShot/frontRPM", 0);
        ChickenSmartDashboard.putDebugNumber("shooter/lastShot/rearRPM", 0);
        ChickenSmartDashboard.putDebugNumber("shooter/lastShot/distanceFromTarget", 0);
        ChickenSmartDashboard.putDebugBoolean("shooter/lastShot/hoodState", false);

        updatePIDs();
    }

    @Override
    public void periodic() {
        ChickenSmartDashboard.putDebugNumber("shooter/current", shooterMotorFront.getStatorCurrent() + shooterMotorRear.getStatorCurrent());
        ChickenSmartDashboard.putDebugNumber("shooter/velocityFront", shooterMotorFront.getVelocityRPM());
        ChickenSmartDashboard.putDebugNumber("shooter/velocityRear", shooterMotorRear.getVelocityRPM());
        ChickenSmartDashboard.putDebugNumber("shooter/error", getClosedLoopError());
        ChickenSmartDashboard.putDebugNumber("shooter/error/front", getFrontClosedLoopError());
        ChickenSmartDashboard.putDebugNumber("shooter/error/rear", getRearClosedLoopError());
        ChickenSmartDashboard.putDebugBoolean("shooter/isSpunUp", isSpunUp());
        ChickenSmartDashboard.putDebugNumber("shooter/frontPercent", shooterMotorFront.getMotorOutputPercent());
        ChickenSmartDashboard.putDebugNumber("shooter/rearPercent", shooterMotorRear.getMotorOutputPercent());
    }

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
        shooterMotorFront.config_kP(0, SmartDashboard.getNumber("shooter/tuning/frontP", frontP));
        shooterMotorFront.config_kI(0, SmartDashboard.getNumber("shooter/tuning/frontI", frontI));
        shooterMotorFront.config_kD(0, SmartDashboard.getNumber("shooter/tuning/frontD", frontD));
        shooterMotorFront.config_kF(0, SmartDashboard.getNumber("shooter/tuning/frontF", frontF));

        shooterMotorRear.config_kP(0, SmartDashboard.getNumber("shooter/tuning/rearP", rearP));
        shooterMotorRear.config_kI(0, SmartDashboard.getNumber("shooter/tuning/rearI", rearI));
        shooterMotorRear.config_kD(0, SmartDashboard.getNumber("shooter/tuning/rearD", rearD));
        shooterMotorRear.config_kF(0, SmartDashboard.getNumber("shooter/tuning/rearF", rearF));
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
        return new InstantCommand(() -> this.setVelocityRPM(front, rear), this);
    }

    /**
     * Check if the shooter is spun up close enough to target velocity
     *
     * @return if the shooter is spun up
     */
    public boolean isSpunUp() {
        return getClosedLoopError() < SmartDashboard.getNumber("shooter/tuning/targetError", 30)
                && Math.abs(shooterMotorFront.getVelocityRPM() + shooterMotorRear.getVelocityRPM()) > 200; // Make sure the shooter is moving
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

        /**
         * Use manual setpoints from SmartDashboard
         */
        TESTING
    }
}
