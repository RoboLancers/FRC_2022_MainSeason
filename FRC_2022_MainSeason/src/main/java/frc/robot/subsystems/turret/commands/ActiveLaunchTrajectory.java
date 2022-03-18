package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.LaunchTrajectory;
import frc.robot.subsystems.turret.Turret;

public class ActiveLaunchTrajectory extends CommandBase {
    private Turret turret;

    public ActiveLaunchTrajectory(Turret turret){
        this.turret = turret;
        this.turret.launchTrajectory = new LaunchTrajectory(0, 0);
        
        this.addRequirements(this.turret);
    }

<<<<<<< HEAD
//     @Override
//     public void execute(){
//         if(this.turret.inHangMode){
//             return;
//         }
//         if(this.yaw.BetterYaw.isAligned()){
//             double distance = LaunchTrajectory.estimateDistance(
//                 Constants.Turret.PhysicsInfo.kTurretShotDeltaY,
//                 Maths.toRadians(this.turret.yaw.limelight.yawOffset()),
//                 Maths.toRadians(this.turret.yaw.limelight.pitchOffset() + Constants.Turret.PhysicsInfo.kPitchMountAngle)
//             );
//             TODO uncomment once the interpolation table has been tuned
//             this.turret.launchTrajectory = LaunchTrajectory.trajectoryMap.interpolate(distance);
//             this.turret.launchTrajectory = LaunchTrajectory.usingAlphaImpact(
//                 Constants.Turret.PhysicsInfo.kGravity,
//                 distance,
//                 Constants.Turret.PhysicsInfo.kTurretShotDeltaY,
//                 Constants.Turret.PhysicsInfo.kSinAlpha,
//                 Constants.Turret.PhysicsInfo.kCosAlpha,
//                 Constants.Turret.PhysicsInfo.kTanAlpha
//             );
//             this.turret.yaw.hasRelativeHub = true;
//             double angle = this.driveTrain.getPose().getRotation().getRadians() + Maths.toRadians(turret.yaw.getPosition());
//             Pose2d robotPose = this.driveTrain.getPose();
//             this.turret.yaw.relativeHub = new Pose2d(robotPose.getX() + Math.cos(angle) * distance, robotPose.getY() + Math.sin(angle) * distance, new Rotation2d());
//         } else if(this.turret.yaw.hasRelativeHub){
//             double deltaX = this.turret.yaw.relativeHub.getX() - this.driveTrain.getPose().getX();
//             double deltaY = this.turret.yaw.relativeHub.getY() - this.driveTrain.getPose().getY();
//             double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
//             if(distance < Constants.Turret.PhysicsInfo.minLimelightViewableDistance || distance > Constants.Turret.PhysicsInfo.maxLimelightViewableDistance){
//                 TODO uncomment once the interpolation table has been tuned
//                 this.turret.launchTrajectory = LaunchTrajectory.trajectoryMap.interpolate(distance);
//                 this.turret.launchTrajectory = LaunchTrajectory.usingAlphaImpact(
//                     Constants.Turret.PhysicsInfo.kGravity,
//                     distance,
//                     Constants.Turret.PhysicsInfo.kTurretShotDeltaY,
//                     Constants.Turret.PhysicsInfo.kSinAlpha,
//                     Constants.Turret.PhysicsInfo.kCosAlpha,
//                     Constants.Turret.PhysicsInfo.kTanAlpha
//                 );
//             }
//         }
//     }
=======
    @Override
    public void execute(){
        this.turret.pitch.setPosition(this.turret.launchTrajectory.theta);

        // force stop if target speed is 0
        if(this.turret.launchTrajectory.speed == 0){
            this.turret.flywheel.setPower(0);
        } else {
            this.turret.flywheel.setVelocity(this.turret.launchTrajectory.speed);
        }        
    }
>>>>>>> 51c0f53f0f499ead1f5f5777d9d603d7599c6767

    @Override
    public boolean isFinished(){
        return false;
    }
}
