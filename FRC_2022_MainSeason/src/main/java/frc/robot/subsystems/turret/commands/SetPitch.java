//Nailah (pls don't remove)
package frc.robot.subsystems.turret.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer; 
//import frc.robot.utilities.XboxController; Don't have anything related to this that I know of
import frc.robot.subsystems.turret.Turret;

public class SetPitch extends CommandBase {
private double pitchangle;
private Turret turret;

public SetPitch(double pitchangle, Turret turret) { //initalizing?? (parameter)
    this.pitchangle = pitchangle;
    this.turret = turret;
}

@Override
public void initialize() {//"set the schedule" ig?

}

@Override
public void execute() { //where the action happens? Is this where I set the angle

}

@Override
public void end(boolean interrupted) {


}

@Override 
public boolean isFinished() { //checks to see if command is finished? 
return pitchangle; //mismatched types; will not work 
return turret; //mismatched typed; will not work 
}

@Override 
public boolean runsWhenDisabled() { //should robot run when disabled? no. 
    return false;
}

    
}
