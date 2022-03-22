package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.LaunchTrajectory;
import frc.robot.subsystems.turret.Turret;

public class LowHubShot extends CommandBase {
    public LowHubShot(Turret turret){
        turret.launchTrajectory = new LaunchTrajectory(12, 1700);

        this.addRequirements(turret);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}