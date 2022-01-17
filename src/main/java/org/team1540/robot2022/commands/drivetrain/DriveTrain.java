package org.team1540.robot2022.commands.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.team1540.robot2022.Constants.DriveTrain.Motors;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
    private final TalonFX driveLFront = new TalonFX(Motors.leftFront);
    private final TalonFX driveLRear = new TalonFX(Motors.leftRear);
    private final TalonFX driveRFront = new TalonFX(Motors.rightFront);
    private final TalonFX driveRRear = new TalonFX(Motors.rightRear);
    private final TalonFX[] driveL = { driveLFront, driveLRear };
    private final TalonFX[] driveR = { driveRFront, driveRRear };
    private final TalonFX[] driveMotors = { driveLFront, driveLRear, driveRFront, driveRRear };

    public DriveTrain(NeutralMode brakeType, NavX navx) {
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

    public void setNeutralMode(NeutralMode mode) {
        for (TalonFX motor : driveMotors) {    
            motor.setNeutralMode(mode);
        }
    }
}
