package frc.robot.subsystems.Climber.commands;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.Climber;

public class MidRung extends CommandBase{
    Climber climber;
    double climbHeight;
    public MidRung(Climber climber, double climbHeight) {
        this.climber = climber;
        this.climbHeight = climbHeight;
    }

    // spelling
    public void extecute(){
        climber.set(Constants.Climber.kPower);
    }

    @Override
    public boolean isFinished(){
        // change to "return climber.getPosition() <= climbHeight"
        // right now it will have an error for not returning false
        if (climber.getPosition() <= (climbHeight)){
            return true;
        }
    }
    
    @Override
    public void end(boolean interrupted){
        climber.set(0);

    }
}
