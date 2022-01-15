package org.team1540.robot2022.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.team1540.robot2022.Constants.DriveMotors;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
    private final TalonFX driveLFront = new TalonFX(DriveMotors.leftFront);
    private final TalonFX driveLRear = new TalonFX(DriveMotors.leftRear);
    private final TalonFX driveRFront = new TalonFX(DriveMotors.rightFront);
    private final TalonFX driveRRear = new TalonFX(DriveMotors.rightRear);
    private final TalonFX[] driveL = { driveLFront, driveLRear };
    private final TalonFX[] driveR = { driveRFront, driveRRear };
    private final TalonFX[] driveMotors = { driveLFront, driveLRear, driveRFront, driveRRear };

    public DriveTrain(NeutralMode brakeType) {
        for (TalonFX motor : driveMotors) {
            motor.configFactoryDefault();
            motor.setNeutralMode(brakeType);
        }
        // Set configuration for left motors
        for (TalonFX motor : driveL) {
            motor.setInverted(true);
        }
        // Set configuration for right motors
        for (TalonFX motor : driveR) {
            motor.setInverted(false);
        }
        driveLRear.follow(driveLFront);
        driveRRear.follow(driveRFront);
    }

    public void setPercent(double leftPercent, double rightPercent) {
        driveLFront.set(ControlMode.PercentOutput, leftPercent);
        driveRFront.set(ControlMode.PercentOutput, rightPercent);
    }
}
