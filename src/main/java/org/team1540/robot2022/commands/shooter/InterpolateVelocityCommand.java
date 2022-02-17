package org.team1540.robot2022.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team1540.robot2022.InterpolationTable;
import org.team1540.robot2022.utils.Limelight;

public class InterpolateVelocityCommand extends CommandBase {
    private final Shooter shooter;
    private final InterpolationTable interpolationTable;
    private final Limelight limelight;

    public InterpolateVelocityCommand(Shooter shooter, InterpolationTable interpolationTable, Limelight limelight) {
        this.shooter = shooter;
        this.interpolationTable = interpolationTable;
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
            // TODO: Set default values and post them to smartdashboard
            shooter.setVelocityRPM(shooter.shooterMotorFront, SmartDashboard.getNumber("shooter/tarmacDefaultFrontRPM", 0));
            shooter.setVelocityRPM(shooter.shooterMotorRear, SmartDashboard.getNumber("shooter/tarmacDefaultRearRPM", 0));
        }
    }
}
