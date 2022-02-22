package org.team1540.robot2022.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import org.team1540.robot2022.commands.drivetrain.Drivetrain;
import org.team1540.robot2022.commands.drivetrain.PointToTarget;
import org.team1540.robot2022.commands.indexer.IndexCommand;
import org.team1540.robot2022.commands.indexer.Indexer;
import org.team1540.robot2022.commands.intake.Intake;
import org.team1540.robot2022.utils.Limelight;
import org.team1540.robot2022.utils.RepeatCommand;

public class ShootSequence extends SequentialCommandGroup {
    private final Command indexCommand;
    private final Shooter shooter;
    private final Indexer indexer;
    private final Drivetrain drivetrain;
    private final Limelight limelight;
    private boolean indexCommandScheduled;

    public ShootSequence(Shooter shooter, Indexer indexer, Drivetrain drivetrain, Limelight limelight, Command indexCommand) {
        this.indexCommand = indexCommand;
        this.shooter = shooter;
        this.indexer = indexer;
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        addRequirements(shooter, indexer, drivetrain);
        applyCommands();
        
    }

    public ShootSequence(Shooter shooter, Indexer indexer, Drivetrain drivetrain, Intake intake, Limelight limelight) {
        this.indexCommand = new RepeatCommand(new IndexCommand(indexer, intake));
        this.shooter = shooter;
        this.indexer = indexer;
        this.drivetrain = drivetrain;
        this.limelight = limelight;
        addRequirements(shooter, indexer, drivetrain);
        applyCommands();
        
    }

    private void applyCommands() {
        addCommands(
                sequence(
                        new InstantCommand(() -> {
                            indexCommandScheduled = indexCommand.isScheduled();
                            indexCommand.cancel();
                        }),
                        parallel(
                                new InstantCommand(() -> {
                                    shooter.setVelocityRPM(shooter.shooterMotorFront, SmartDashboard.getNumber("shooter/tuning/frontRPM", 0));
                                    shooter.setVelocityRPM(shooter.shooterMotorRear, SmartDashboard.getNumber("shooter/tuning/rearRPM", 0));
                                }, shooter),
//                                new InterpolateVelocityCommand(shooter, limelight),
                                new ConditionalCommand( // Shoot if target isn't found, otherwise lineup and shoot
                                        new PointToTarget(drivetrain, limelight),
                                        new InstantCommand(),
                                        limelight::isTargetFound
                                )
                        ),
                        new WaitCommand(1),
                        new WaitUntilCommand(shooter::isSpunUp),
                        new InstantCommand(() -> indexer.set(Indexer.IndexerState.FORWARD_FULL, Indexer.IndexerState.FORWARD_FULL)),
                        new WaitCommand(2), // TODO: Maybe fix this?
                        new InstantCommand(() -> {
                            indexer.set(Indexer.IndexerState.OFF, Indexer.IndexerState.OFF);
                            shooter.setVelocityRPM(shooter.shooterMotorFront, 0);
                            shooter.setVelocityRPM(shooter.shooterMotorRear, 0);
                        })
                )
        );
    }
    @Override
    public void end(boolean isInterrupted) {
        System.out.println("Stopping ShootSequence");
        shooter.stop();
        indexer.stop();

        if (indexCommandScheduled) {
            indexCommand.schedule();
        }
    }
}
