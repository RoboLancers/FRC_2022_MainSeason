package frc.robot.subsystems.turret.commands;
 
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.turret.LaunchTrajectory;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.Maths;
 
public class ActiveLaunchTrajectory extends CommandBase {
    private Turret turret;
 
    public ActiveLaunchTrajectory(Turret turret){
        this.turret = turret;
 
        addRequirements(turret);
    }
 
    @Override
    public void execute(){
        if(this.turret.yaw.limelight.hasTarget()){
            // TODO: use pass through trajectory if outside of a certain distance
            this.turret.launchTrajectory = LaunchTrajectory.usingAlphaImpact(
                Constants.Turret.PhysicsInfo.kGravity,
                LaunchTrajectory.estimateDistance(
                    Constants.Turret.PhysicsInfo.kTurretShotDeltaY,
                    Maths.toRadians(this.turret.yaw.limelight.yawOffset()),
                    Maths.toRadians(this.turret.yaw.limelight.pitchOffset() + Constants.Turret.PhysicsInfo.kPitchMountAngle)
                ),
                Constants.Turret.PhysicsInfo.kTurretShotDeltaY,
                Constants.Turret.PhysicsInfo.kSinAlpha,
                Constants.Turret.PhysicsInfo.kCosAlpha,
                Constants.Turret.PhysicsInfo.kTanAlpha
            );
        };
    }
 
    @Override
    public boolean isFinished(){
        return false;
    }
}