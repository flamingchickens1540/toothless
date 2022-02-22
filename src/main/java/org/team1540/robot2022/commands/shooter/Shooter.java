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

    // Result of the last shot
    public ShootingState lastShot;

    public TalonFX shooterMotorFront = new TalonFX(Constants.ShooterConstants.front);
    public TalonFX shooterMotorRear = new TalonFX(Constants.ShooterConstants.rear);

    public Shooter() {
        Constants.ShooterConstants.currentLimitConfig.applyTo(new TalonFX[]{shooterMotorFront, shooterMotorRear});
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
        return getFrontClosedLoopError() + getRearClosedLoopError();
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

    /**
     * Check if the shooter is spun up close enough to target velocity
     *
     * @return if the shooter is spun up
     */
    public boolean isSpunUp() {
        return Math.abs(getClosedLoopError()) < SmartDashboard.getNumber("shooter/tuning/targetError", 0)
                && Math.abs(getVelocityRPM(shooterMotorFront) + getVelocityRPM(shooterMotorRear)) > 200; // Make sure the shooter is moving
    }

    /**
     * Record a shot
     *
     * @param frontVelocity  front shooter flywheel RPM
     * @param rearVelocity   rear shooter flywheel RPM
     * @param targetDistance target distance in inches
     * @param hood           is the hood up?
     */
    public void recordShot(double frontVelocity, double rearVelocity, double targetDistance, boolean hood) {
        // If the last shot hasn't been saved yet, save it with an unknown value
        // (setLastShotResult sets lastShot to null when it's done recording)
        if (lastShot != null) {
            setLastShotResult(ShotResult.UNKNOWN);
        }

        lastShot = new ShootingState(frontVelocity, rearVelocity, targetDistance, hood);
    }

    /**
     * Record the result of the last shot and save it
     *
     * @param result shot result
     */
    public void setLastShotResult(ShotResult result) {
        if (lastShot != null) {
            lastShot.result = result;
            saveShotToFile(lastShot);
            lastShot = null;
        }
    }

    /**
     * Save a shot in the optimization table
     */
    private void saveShotToFile(ShootingState shot) {
        String jsonl = String.format("{'match': %d, 'replay': %d, 'alliance': '%s', 'location': %d, 'matchSeconds': %f," +
                        "'frontRPM': %f, 'rearRPM': %f, 'targetDistance': %f, 'hood': %b, 'result': '%s'}",
                DriverStation.getMatchNumber(),
                DriverStation.getReplayNumber(),
                DriverStation.getAlliance().toString(),
                DriverStation.getLocation(),
                DriverStation.getMatchTime(),
                shot.frontVelocity,
                shot.rearVelocity,
                shot.targetDistance,
                shot.hood,
                shot.result);

        try {
            Files.write(Paths.get("/optimization.jsonl"), jsonl.getBytes(), StandardOpenOption.APPEND);
        } catch (IOException e) {
            DriverStation.reportError("ERROR: While appending to optimization file: " + e, e.getStackTrace());
        }
    }
}
