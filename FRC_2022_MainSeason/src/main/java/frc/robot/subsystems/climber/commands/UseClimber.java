package frc.robot.subsystems.Climber.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.Climber;

public class UseClimber extends CommandBase{
    Climber climber;

    public UseClimber(Climber climber) {
        this.climber = climber;
    }
}
