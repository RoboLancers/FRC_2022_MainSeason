package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.climber.Climber;

public class UseClimber extends CommandBase{
    Climber climber;

    public UseClimber(Climber climber) {
        this.climber = climber;
    }


    
}
