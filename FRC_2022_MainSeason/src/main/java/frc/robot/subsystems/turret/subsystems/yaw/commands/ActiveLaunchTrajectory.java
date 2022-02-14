package frc.robot.subsystems.turret.subsystems.yaw.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.turret.LaunchTrajectory;
import frc.robot.subsystems.turret.subsystems.yaw.TurretYaw;
import frc.robot.util.Maths;

public class ActiveLaunchTrajectory extends CommandBase {
    private TurretYaw turretYaw;

    public ActiveLaunchTrajectory(TurretYaw turretYaw){
        this.turretYaw = turretYaw;
    }

    @Override
    public void execute(){
        if(this.turretYaw.limelight.hasTarget()){
            // TODO: use pass through trajectory if outside of a certain distance
            this.turretYaw.updateLaunchTrajectory(LaunchTrajectory.usingAlphaImpact(
                Constants.Turret.PhysicsInfo.kGravity,
                LaunchTrajectory.estimateDistance(
                    Constants.Turret.PhysicsInfo.kTurretShotDeltaY,
                    Maths.toRadians(this.turretYaw.limelight.yawOffset()),
                    Maths.toRadians(this.turretYaw.limelight.pitchOffset() + Constants.Turret.PhysicsInfo.kPitchMountAngle)
                ),
                Constants.Turret.PhysicsInfo.kTurretShotDeltaY,
                Constants.Turret.PhysicsInfo.kSinAlpha,
                Constants.Turret.PhysicsInfo.kCosAlpha,
                Constants.Turret.PhysicsInfo.kTanAlpha
            ));
        };
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
