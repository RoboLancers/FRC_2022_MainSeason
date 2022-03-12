package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.turret.LaunchTrajectory;
import frc.robot.subsystems.turret.Turret;

public class ActiveLaunchTrajectory extends CommandBase {
    private Turret turret;

    public ActiveLaunchTrajectory(Turret turret){
        this.turret = turret;
        
        this.addRequirements(this.turret);
    }

    @Override
    public void execute(){
        if(this.turret.inHangMode){
            this.turret.launchTrajectory = new LaunchTrajectory(0, 0);
            return;
        }
        double distance;
        if(this.turret.limelight.hasTarget()){
            distance = LaunchTrajectory.estimateDistance(this.turret.limelight.pitchOffset());
        } else {
            distance = Constants.Turret.Physics.kUpperHubRadius;
        };
        this.turret.launchTrajectory = LaunchTrajectory.trajectoryMap.interpolate(distance);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
