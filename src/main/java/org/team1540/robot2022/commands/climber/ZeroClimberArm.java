package org.team1540.robot2022.commands.climber;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

// This command brings a single arm to its lower limit and resets the encoder to zero
public class ZeroClimberArm extends SequentialCommandGroup{

    TalonFX motor;
    static final double LOWER_VEL = 0; //TODO: Tune
    static final double VOLTAGE_THRESHOLD = 0; //TODO: Tune

    public ZeroClimberArm(Climber climber, Climber.ArmSide side) {
        this.motor = side.getMotor(climber);

        addCommands(
            new InstantCommand(()->climber.setClimberVelocity(side, LOWER_VEL)),
            new WaitUntilCommand(()->climber.getArmVoltage(side) > VOLTAGE_THRESHOLD),
            new InstantCommand(()->{
                motor.setSelectedSensorPosition(0, 0, 0);
                climber.setClimberVelocity(side, 0);
            })
        );
    }

}
