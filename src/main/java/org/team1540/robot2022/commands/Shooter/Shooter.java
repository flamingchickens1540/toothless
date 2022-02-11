package org.team1540.robot2022.commands.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team1540.robot2022.LunaUtils.ControlUtils;
import org.team1540.robot2022.LunaUtils.MotorConfigUtils;
import org.team1540.robot2022.LunaUtils.Timer;

import edu.wpi.first.wpilibj2.command.button.getLeftBumper;
import edu.wpi.first.wpilibj2.command.button.getRightBumper;



//Add commands 

public class Shooter extends SubsystemBase {
   //private ShooterTesting shooter = new ShooterTesting();

    private double kP = 0.5;
    private double kD = 30;
    private double kF = 0.0484;
    private double kI = 0;
    private double iZone = 0;

    private TalonFX shooterMotorA = new TalonFX(9);
    private TalonFX shooterMotorB = new TalonFX(10);

   // public static final int HIGH_RPM_kD = 30;
    //public static final int LOW_RPM_kD = 10;
    private Timer kdTimer = new Timer();

    public Shooter() {
        setupFlywheelMotors();
        setupPIDs();
        updatePIDs();
        kdTimer.reset();
        kdTimer.start();
    }

    private void setupFlywheelMotors() {
        MotorConfigUtils.setDefaultTalonFXConfig(shooterMotorA);
        MotorConfigUtils.setDefaultTalonFXConfig(shooterMotorB);
        shooterMotorA.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 0, 0, 0));
        shooterMotorB.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 0, 0, 0));
        shooterMotorA.setNeutralMode(NeutralMode.Coast);
        shooterMotorB.setNeutralMode(NeutralMode.Coast);
//        shooterMotorA.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_1Ms);
//        shooterMotorA.configVelocityMeasurementWindow(4);


        shooterMotorB.setInverted(TalonFXInvertType.OpposeMaster);
    }

    private void setupPIDs() {
        SmartDashboard.putNumber("shooter/tuning/kP", kP);
        SmartDashboard.putNumber("shooter/tuning/kI", kI);
        SmartDashboard.putNumber("shooter/tuning/iZone", iZone);
//        SmartDashboard.putNumber("shooter/tuning/kD", kD);
        SmartDashboard.putNumber("shooter/tuning/kF", kF);
        SmartDashboard.putNumber("shooter/tuning/kD", kD);


        NetworkTableInstance.getDefault().getTable("SmartDashboard/shooter/tuning").addEntryListener((table, key, entry, value, flags) -> updatePIDs(), EntryListenerFlags.kUpdate);
    }

    //@Override
    public void periodic() {
        SmartDashboard.putNumber("shooter/current", shooterMotorA.getStatorCurrent() + shooterMotorB.getStatorCurrent());
        SmartDashboard.putNumber("shooter/velocityA", getVelocityRPMA());
        SmartDashboard.putNumber("shooter/velocityB", getVelocityRPMB());
        SmartDashboard.putNumber("shooter/error", getClosedLoopError());
        SmartDashboard.putNumber("setVelocityRPMA", setVelocityRPMA(500));
        SmartDashboard.putNumber("setVelocityRPMB", setVelocityRPMB(500));

    }


    /*private void configLowRPM(double targetRPM) {
        double kD = ControlUtils.linearDeadzoneRamp(targetRPM, false, HIGH_RPM_kD, LOW_RPM_kD, 5000, 3000);
        config_kD(kD);
        SmartDashboard.putNumber("shooter/calculatedKd", kD);
    }
    */

    public void stop() {
        shooterMotorA.set(TalonFXControlMode.PercentOutput, 0);
        shooterMotorB.set(TalonFXControlMode.PercentOutput, 0);
    }

    public double getVelocityRPMA() {
        return (shooterMotorA.getSelectedSensorVelocity() / 2048.0) * 600;
    }
    public double getVelocityRPMB() {
        return (shooterMotorB.getSelectedSensorVelocity() / 2048.0) * 600;
    }

    public double setVelocityRPMA(double velocity) {
        //if (kdTimer.hasPeriodPassed(0.3)) configLowRPM(velocity);
        shooterMotorA.set(TalonFXControlMode.Velocity, (velocity * 2048.0) / 600);
        return velocity; 
    }
    public double setVelocityRPMB(double velocity) {
        if (kdTimer.hasPeriodPassed(0.3)) configLowRPM(velocity);
        shooterMotorA.set(TalonFXControlMode.Velocity, (velocity * 2048.0) / 600);
        return velocity; 
    }

    private void updatePIDs() {
        shooterMotorA.config_kP(0, SmartDashboard.getNumber("shooter/tuning/kP", kP));
        shooterMotorA.config_kI(0, SmartDashboard.getNumber("shooter/tuning/kI", kI));
//        shooterMotorA.config_kD(0, SmartDashboard.getNumber("shooter/tuning/kD", kD));
        shooterMotorA.config_kF(0, SmartDashboard.getNumber("shooter/tuning/kF", kF));
        shooterMotorA.config_IntegralZone(0, (int) SmartDashboard.getNumber("shooter/tuning/iZone", iZone));
    }

    public void setPercent(double value) {
        shooterMotorA.set(ControlMode.PercentOutput, value);
        shooterMotorB.set(ControlMode.PercentOutput, value);

    }

    public double getClosedLoopError() {
        return shooterMotorA.getClosedLoopError() + shooterMotorB.getClosedLoopError();
    }

    public void config_kD(double kD) {
        shooterMotorA.config_kD(0, kD);
        shooterMotorB.config_kD(0, kD);

    }

    public void setShooterMotorA(){

    }

    public Command commandStop() {
        return new InstantCommand(this::stop, this);
    }

  /* public Command commandStartFlyWheelA(){
       //return new RunCommand(() -> shooter.setVelocityRPMA(
           
       
       //return setVelocityRPMA(0.3); 
   }
   */ 

   public void startFlyWheelA(){
       new RightBumper(this::setVelocityRPMA)
            .whileActiveContinuous(setRPMA); 
   }

   public void startFlyWheelB(){
    new LeftBumper(this::setVelocityRPMB)
            .whileActiveContinuous(setRPMB); 
   }

    
    
}


