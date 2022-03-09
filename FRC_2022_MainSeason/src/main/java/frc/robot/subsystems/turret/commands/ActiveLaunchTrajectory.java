package frc.robot.subsystems.turret.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.turret.LaunchTrajectory;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.Maths;

public class ActiveLaunchTrajectory extends CommandBase {
    private Turret turret;
    private Drivetrain driveTrain;

    public ActiveLaunchTrajectory(Turret turret, Drivetrain driveTrain){
        this.turret = turret;
        this.driveTrain = driveTrain;
        
        this.addRequirements(this.turret);
    }

    @Override
    public void execute(){
        if(this.turret.inHangMode){
            return;
        }
        if(this.turret.yaw.limelight.hasTarget()){
            double distance = LaunchTrajectory.estimateDistance(
                Constants.Turret.PhysicsInfo.kTurretShotDeltaY,
                Maths.toRadians(this.turret.yaw.limelight.pitchOffset() + Constants.Turret.PhysicsInfo.kPitchMountAngle)
            );
            this.turret.launchTrajectory = LaunchTrajectory.trajectoryMap.interpolate(distance);
            this.turret.yaw.hasRelativeHub = true;
            double angle = this.driveTrain.getPose().getRotation().getRadians() + Maths.toRadians(turret.yaw.getPosition());
            Pose2d robotPose = this.driveTrain.getPose();
            this.turret.yaw.relativeHub = new Pose2d(robotPose.getX() + Math.cos(angle) * distance, robotPose.getY() + Math.sin(angle) * distance, new Rotation2d());
        } else if(this.turret.yaw.hasRelativeHub){
            double deltaX = this.turret.yaw.relativeHub.getX() - this.driveTrain.getPose().getX();
            double deltaY = this.turret.yaw.relativeHub.getY() - this.driveTrain.getPose().getY();
            double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
            if(distance < Constants.Turret.PhysicsInfo.minLimelightViewableDistance || distance > Constants.Turret.PhysicsInfo.maxLimelightViewableDistance){
                this.turret.launchTrajectory = LaunchTrajectory.trajectoryMap.interpolate(distance);
            }
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
