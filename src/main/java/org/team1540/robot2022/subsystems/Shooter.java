package org.team1540.robot2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.team1540.robot2022.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    public TalonFX motor1, motor2;

    public Shooter() {
        motor1 = new TalonFX(Constants.SHOOTER_CAN_1);
        motor2 = new TalonFX(Constants.SHOOTER_CAN_2);
        // Make more dynamic than this
        motor2.follow(motor1);
    }
}
