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
            return;
        }
        if(this.turret.limelight.hasTarget()){
            double distance = LaunchTrajectory.estimateDistance(
                Constants.Turret.PhysicsInfo.kDeltaY,
                this.turret.limelight.pitchOffset() + Constants.Turret.PhysicsInfo.kMountAngle
            );
            this.turret.setLaunchTrajectory(LaunchTrajectory.trajectoryMap.interpolate(distance));
        } else {
            this.turret.resetLaunchTrajectory();
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
