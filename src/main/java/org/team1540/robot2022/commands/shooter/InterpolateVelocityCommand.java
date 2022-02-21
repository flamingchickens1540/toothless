package org.team1540.robot2022.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2022.InterpolationTable;
import org.team1540.robot2022.utils.Limelight;

public class InterpolateVelocityCommand extends CommandBase {
    private final Shooter shooter;
    private final InterpolationTable interpolationTable = InterpolationTable.getInstance();;
    private final Limelight limelight;

    public InterpolateVelocityCommand(Shooter shooter, Limelight limelight) {
        this.shooter = shooter;
        this.limelight = limelight;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        if (limelight.isTargetFound()) {
            double distanceFromTarget = limelight.getCalculatedDistance();
            shooter.setVelocityRPM(shooter.shooterMotorFront, interpolationTable.frontFlywheelInterpolator.getInterpolatedValue(distanceFromTarget));
            shooter.setVelocityRPM(shooter.shooterMotorRear, interpolationTable.rearFlywheelInterpolator.getInterpolatedValue(distanceFromTarget));
        } else { // Limelight target not found
            shooter.setVelocityRPM(shooter.shooterMotorFront, -2000);
            shooter.setVelocityRPM(shooter.shooterMotorRear, -2000);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
