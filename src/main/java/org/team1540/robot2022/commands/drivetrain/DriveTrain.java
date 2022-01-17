package org.team1540.robot2022.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.team1540.robot2022.Constants.DriveTrain.Motors;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
    private final ChickenTalonFX driveLFront = new ChickenTalonFX(Motors.leftFront);
    private final ChickenTalonFX driveLRear = new ChickenTalonFX(Motors.leftRear);
    private final ChickenTalonFX driveRFront = new ChickenTalonFX(Motors.rightFront);
    private final ChickenTalonFX driveRRear = new ChickenTalonFX(Motors.rightRear);
    private final ChickenTalonFX[] driveL = { driveLFront, driveLRear };
    private final ChickenTalonFX[] driveR = { driveRFront, driveRRear };
    private final ChickenTalonFX[] driveMotors = { driveLFront, driveLRear, driveRFront, driveRRear };

    public DriveTrain(NeutralMode brakeType, NavX navx) {
        for (ChickenTalonFX motor : driveMotors) {
            motor.configFactoryDefault();
            motor.setNeutralMode(brakeType);
        }
        // Set configuration for left motors
        for (ChickenTalonFX motor : driveL) {
            motor.setInverted(true);
        }
        // Set configuration for right motors
        for (ChickenTalonFX motor : driveR) {
            motor.setInverted(false);
        }
        driveLRear.follow(driveLFront);
        driveRRear.follow(driveRFront);
    }

    public void setPercent(double leftPercent, double rightPercent) {
        driveLFront.set(ControlMode.PercentOutput, leftPercent);
        driveRFront.set(ControlMode.PercentOutput, rightPercent);
    }

    public void setNeutralMode(NeutralMode mode) {
        for (ChickenTalonFX motor : driveMotors) {
            motor.setNeutralMode(mode);
        }
    }
}
