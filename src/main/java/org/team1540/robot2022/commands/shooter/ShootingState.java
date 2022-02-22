package org.team1540.robot2022.commands.shooter;

/**
 * ShootingState holds shooter parameters and result
 */
public class ShootingState {
    public double frontVelocity;
    public double rearVelocity;
    public double targetDistance;
    public boolean hood;
    public ShotResult result;

    /**
     * Save a shot in the optimization table
     *
     * @param frontVelocity  front shooter flywheel RPM
     * @param rearVelocity   rear shooter flywheel RPM
     * @param targetDistance target distance in inches
     * @param hood           is the hood up?
     */
    public ShootingState(double frontVelocity, double rearVelocity, double targetDistance, boolean hood) {
        this.frontVelocity = frontVelocity;
        this.rearVelocity = rearVelocity;
        this.targetDistance = targetDistance;
        this.hood = hood;
    }
}
