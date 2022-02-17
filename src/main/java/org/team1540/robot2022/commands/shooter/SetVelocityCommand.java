package org.team1540.robot2022.commands.shooter;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * SetVelocityCommand sets a static shooter RPM
 */
public class SetVelocityCommand extends CommandBase {
    private final Shooter shooter;
    private final double frontVelocity;
    private final double rearVelocity;

    public SetVelocityCommand(Shooter shooter, double frontVelocity, double rearVelocity) {
        this.shooter = shooter;
        this.frontVelocity = frontVelocity;
        this.rearVelocity = rearVelocity;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.shooterMotorFront.set(TalonFXControlMode.Velocity, frontVelocity);
        shooter.shooterMotorRear.set(TalonFXControlMode.Velocity, rearVelocity);
    }
}
