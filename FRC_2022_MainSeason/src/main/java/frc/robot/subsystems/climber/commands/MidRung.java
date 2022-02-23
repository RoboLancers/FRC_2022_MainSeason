package frc.robot.subsystems.climber.commands;
import frc.robot.Constants;
import frc.robot.subsystems.climber.Climber;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class MidRung extends CommandBase{
    Climber climber;
    double climbHeight;
    public MidRung(Climber climber, double climbHeight) {
        this.climber = climber;
        this.climbHeight = climbHeight;
    }

<<<<<<< HEAD
    // spelling
=======
    
>>>>>>> 6202e94a8999bdb9e8df53905f3d83cbdbbd41a4
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
