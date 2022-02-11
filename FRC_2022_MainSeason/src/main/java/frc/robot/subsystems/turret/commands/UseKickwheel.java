<<<<<<< HEAD
//Nailah 
package frc.robot.subsystems.turret.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;  
//import frc.robot.utilities.XboxController; Don't have anything related to this that I know of
import frc.robot.subsystems.turret.Turret;


public class UseKickwheel extends CommandBase  {

private double usekickwheel;
private Turret turret;

public UseKickwheel(double usekickwheel, Turret turret) {
 this.usekickwheel = usekickwheel;
 this.turret = turret;  
} 

@Override 
public void initialize() {

}

@Override
public void execute() {
    
}

@Override
public void end(boolean interrupted) {

}

@Override 
public boolean isFinished(){
    return true;  
}
@Override 
public boolean runsWhenDisabled() {
    return false;
} 
    
=======
package frc.robot.subsystems.turret.commands;

public class UseKickwheel {
    // TODO: NS
>>>>>>> a210acf3ac7b2b4373907f22e36a7544d082f006
}

