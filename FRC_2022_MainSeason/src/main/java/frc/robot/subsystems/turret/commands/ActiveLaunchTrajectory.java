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
        if(this.turret.limelight.hasTarget()){
            this.turret.launchTrajectory = LaunchTrajectory.trajectoryMap.interpolate(
                LaunchTrajectory.estimateDistance(this.turret.limelight.pitchOffset())
            );
        } else {
            // TODO: use odemetry to rev in this situation
            this.turret.launchTrajectory = new LaunchTrajectory(0, 0);
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
