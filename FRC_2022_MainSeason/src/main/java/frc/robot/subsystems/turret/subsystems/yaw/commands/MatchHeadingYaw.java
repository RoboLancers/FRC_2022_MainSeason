package frc.robot.subsystems.turret.subsystems.yaw.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.turret.subsystems.yaw.TurretYaw;

public class MatchHeadingYaw extends CommandBase {
    private TurretYaw turretYaw;
    private Drivetrain driveTrain;

    private boolean seekDirection = true;

    public MatchHeadingYaw(TurretYaw turretYaw, Drivetrain driveTrain){
        this.turretYaw = turretYaw;
        this.driveTrain = driveTrain;

        addRequirements(turretYaw);
    }

    @Override
    public void execute(){
        if(this.turretYaw.limelight.hasTarget()){
            double targetYaw = this.turretYaw.getPosition() + this.turretYaw.limelight.yawOffset();
            if(
                targetYaw < Constants.Turret.TunedCoefficients.YawPID.kMinSafeAngle ||
                targetYaw > Constants.Turret.TunedCoefficients.YawPID.kMaxSafeAngle
            ){
                this.seekDirection = targetYaw < Constants.Turret.TunedCoefficients.YawPID.kMinSafeAngle;
                this.turretYaw.setVelocitySetpoint(
                    this.seekDirection ?
                        Constants.Turret.TunedCoefficients.YawPID.kSeekAdjustment :
                        -Constants.Turret.TunedCoefficients.YawPID.kSeekAdjustment
                );
            } else {
                this.turretYaw.setPositionSetpoint(targetYaw);
            }
        } else {
            if(this.turretYaw.hasRelativeHub){
                double deltaX = this.turretYaw.relativeHub.getX() - this.driveTrain.getPose().getX();
                double deltaY = this.turretYaw.relativeHub.getY() - this.driveTrain.getPose().getY();
                double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
                if(distance > Constants.Turret.PhysicsInfo.minLimelightViewableDistance && distance < Constants.Turret.PhysicsInfo.maxLimelightViewableDistance){
                    this.turretYaw.setVelocitySetpoint(
                        this.seekDirection ?
                            Constants.Turret.TunedCoefficients.YawPID.kSeekAdjustment :
                            -Constants.Turret.TunedCoefficients.YawPID.kSeekAdjustment
                    );
                    return;
                }
            }
            this.turretYaw.setPositionSetpoint(Math.atan2(
                this.turretYaw.relativeHub.getY() - this.driveTrain.getPose().getY(),
                this.turretYaw.relativeHub.getX() - this.driveTrain.getPose().getX()
            ));
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
