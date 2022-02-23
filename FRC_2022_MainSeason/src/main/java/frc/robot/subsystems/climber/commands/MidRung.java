package frc.robot.subsystems.climber.commands;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;

public class MidRung extends CommandBase{
    Climber climber;
    double climbHeight;
    public MidRung(Climber climber, double climbHeight) {
        this.climber = climber;
        this.climbHeight = climbHeight;
    }

    // spelling
    
    public void execute(){
        climber.set(Constants.Climber.kPower);
    }

    @Override
    public boolean isFinished(){
        if (climber.getPosition() <= (climbHeight)){
            return true;
        }
        return false;
    }
    
    @Override
    public void end(boolean interrupted){
        climber.set(0);

    }
}
