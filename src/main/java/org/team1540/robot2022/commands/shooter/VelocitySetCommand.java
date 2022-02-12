package org.team1540.robot2022.commands.shooter;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2022.utils.InterpolationTable;
import org.team1540.robot2022.utils.Limelight;

public class VelocitySetCommand extends CommandBase {
    private final Shooter shooter;
    private final InterpolationTable interpolationTable;
    private final Limelight limelight;

    public VelocitySetCommand(Shooter shooter, InterpolationTable interpolationTable, Limelight limelight) {
        this.shooter = shooter;
        this.interpolationTable = interpolationTable;
        this.limelight = limelight;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooter.shooterMotorFront.set(
            TalonFXControlMode.Velocity,
            (interpolationTable.frontFlywheelInterpolator.getInterpolatedValue(limelight.getCalculatedDistance()) * 2048.0) / 600
        );

        shooter.shooterMotorRear.set(
            TalonFXControlMode.Velocity,
            (interpolationTable.rearFlywheelInterpolator.getInterpolatedValue(limelight.getCalculatedDistance()) * 2048.0) / 600
        );
    }
}
