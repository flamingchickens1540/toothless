package org.team1540.robot2022.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// TODO: maybe make this an inline command in RobotWrapper
public class ZeroClimber extends ParallelCommandGroup{

    public ZeroClimber(Climber climber) {
        addCommands(
            new ZeroClimberArm(climber,Climber.ArmSide.LEFT),
            new ZeroClimberArm(climber,Climber.ArmSide.RIGHT)
        );
    }
}
