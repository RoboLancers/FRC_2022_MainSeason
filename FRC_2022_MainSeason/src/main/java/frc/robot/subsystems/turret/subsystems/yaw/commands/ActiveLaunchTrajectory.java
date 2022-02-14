package frc.robot.subsystems.turret.subsystems.yaw.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.turret.LaunchTrajectory;
import frc.robot.subsystems.turret.subsystems.yaw.TurretYaw;
import frc.robot.util.Maths;

public class ActiveLaunchTrajectory extends CommandBase {
    private TurretYaw turretYaw;

    private double lastYaw;

    public ActiveLaunchTrajectory(TurretYaw turretYaw){
        this.turretYaw = turretYaw;

        this.lastYaw = this.turretYaw.getPosition();
    }

    @Override
    public void execute(){
        if(Math.abs(this.turretYaw.getPosition() - lastYaw) > Constants.Turret.PhysicsInfo.kDeltaYawRecalculationThreshold){
            this.lastYaw = this.turretYaw.getPosition();
            if(this.turretYaw.limelight.hasTarget()){
                this.turretYaw.updateLaunchTrajectory(LaunchTrajectory.usingAlphaImpact(
                    Constants.Turret.PhysicsInfo.kGravity,
                    LaunchTrajectory.estimateDistance(
                        Constants.Turret.PhysicsInfo.kTurretShotDeltaY,
                        Maths.toRadians(this.turretYaw.limelight.yawOffset()),
                        Maths.toRadians(this.turretYaw.limelight.pitchOffset() + Constants.Turret.PhysicsInfo.kPitchMountAngle)
                    ),
                    Constants.Turret.PhysicsInfo.kTurretShotDeltaY,
                    Constants.Turret.PhysicsInfo.kAlpha
                ));
            } else {
                this.turretYaw.updateLaunchTrajectory(new LaunchTrajectory(0, 0, 0));
            }
        }

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
