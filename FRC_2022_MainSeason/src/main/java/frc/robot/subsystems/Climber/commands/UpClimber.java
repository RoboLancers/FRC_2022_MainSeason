package frc.robot.subsystems.climber.commands;
import frc.robot.Constants;
import frc.robot.subsystems.climber.Climber;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class UpClimber extends CommandBase{
    private double climbHeight;
    private Climber climber;

    public UpClimber(Climber climber, double climbHeight) {
        this.climber = climber;
        this.climbHeight = climbHeight;
    }

    @Override
    public void execute(){
        if (climber.climberMotor1.getEncoder().getPosition() < (climbHeight)){
            climber.climberMotor1.set(Constants.Climber.kPower);
        }
        else{
            climber.climberMotor1.set(0);
        }
        if (climber.climberMotor2.getEncoder().getPosition() < (climbHeight)){
            climber.climberMotor2.set(Constants.Climber.kPower);
        }
        else{
            climber.climberMotor2.set(0);
        }
    }

    @Override
    public boolean isFinished(){
        return (climber.climberMotor2.getEncoder().getPosition() >= (climbHeight) && climber.climberMotor2.getEncoder().getPosition() >= (climbHeight));
    }
    
    @Override
    public void end(boolean interrupted){
        climber.climberMotor1.set(0);
        climber.climberMotor2.set(0);
    }
}