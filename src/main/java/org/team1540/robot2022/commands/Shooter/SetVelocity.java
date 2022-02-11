package main.java.org.team1540.robot2022.commands.shooter;

public class SetVelocity extends CommandBase {
    private final Shooter shooter; 
    public  SetVelocity(Shooter shooter, LinearInterperlator interperlator, TalonFX flywheel){
        this.shooter = shooter; 
        this.interperlator = interperlator; 
        this.flywheel = flywheel; 
        addRequirements(shooter); 
    }
    
    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
                flywheel.set(TalonFXControlMode.Velocity, (velocity * 2048.0) / 600);
    }
}
