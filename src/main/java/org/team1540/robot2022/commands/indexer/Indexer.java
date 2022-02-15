package org.team1540.robot2022.commands.indexer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.team1540.robot2022.Constants.IndexerConstants;
import org.team1540.robot2022.Constants.IndexerConstants.BeamBreaks;
import org.team1540.robot2022.Constants.IndexerConstants.IndexerMotors;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private final TalonFX bottomMotor = new TalonFX(IndexerMotors.bottomMotor);
    private final TalonFX topMotor = new TalonFX(IndexerMotors.topMotor);
    private final TalonFX[] motors = { topMotor, bottomMotor};

    private final DigitalInput topSensor = new DigitalInput(BeamBreaks.topIndexerSensor);
    private final DigitalInput bottomSensor = new DigitalInput(BeamBreaks.bottomIndexerSensor);

    public Indexer(NeutralMode brakeType) {
        IndexerMotors.currentLimitConfig.applyTo(motors);
        for (TalonFX motor:motors) {
            motor.setNeutralMode(brakeType);
            motor.setInverted(true);
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("indexer/top", this.getTopSensor());
        SmartDashboard.putBoolean("indexer/bottom", this.getBottomSensor());
        SmartDashboard.putBoolean("indexer/full", this.isFull());
    }

    /**
     * Returns true if the top beam break is blocked
     * @return if the sensor is blocked
     */
    public boolean getTopSensor() {
        return !topSensor.get();
    }

    /**
     * Returns true if the bottom beam break is blocked
     * @return if the sensor is blocked
     */
    public boolean getBottomSensor() {
        return !bottomSensor.get();
    }

    public boolean isFull() {
        return (this.getBottomSensor() && this.getTopSensor());
    }
    
    /**
     * Sets the top and bottom parts of the indexer active/
     * @param topOn If the top motors should turn on
     * @param bottomOn If the bottom motors should turn on
     */
    public void set(boolean topOn, boolean bottomOn) {
        this.setTop(topOn);
        this.setBottom(bottomOn);
    }

    public void setTop(boolean topOn) {
        topMotor.set(ControlMode.PercentOutput, 
                (topOn ? IndexerConstants.topPercent : 0)
        );
    }

    public void setBottom(boolean bottomOn) {
        bottomMotor.set(ControlMode.PercentOutput, 
                (bottomOn ? IndexerConstants.bottomPercent : 0)
        );
    }

    public Command commandSetTop(boolean topOn) {
        return new InstantCommand(() -> {
            this.setTop(topOn);
        });
    }
    public Command commandSetBottom(boolean bottomOn) {
        return new InstantCommand(() -> {
            this.setBottom(bottomOn);
        });
    }
    
    public Command commandSet(boolean topOn, boolean bottomOn) {
        return new InstantCommand(() -> {
            this.set(topOn, bottomOn);
        });
    }

    public Command commandStop() {
        return new InstantCommand(() -> {
            System.out.println("CommandStop");
            this.set(false,false);});
    }

}