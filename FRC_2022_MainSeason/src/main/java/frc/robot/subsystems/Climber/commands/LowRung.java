package frc.robot.subsystems.climber.commands;

import frc.robot.subsystems.climber.Climber;
import frc.robot.Constants;
import frc.robot.subsystems.climber.Climber;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LowRung extends CommandBase{
    Climber climber;
    double climbHeight;
    public LowRung(Climber climber, double climbHeight) {
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