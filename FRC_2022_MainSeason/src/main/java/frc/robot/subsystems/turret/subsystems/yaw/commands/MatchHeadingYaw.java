package frc.robot.subsystems.turret.subsystems.yaw.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.turret.subsystems.yaw.TurretYaw;

public class MatchHeadingYaw extends CommandBase {
    private TurretYaw turretYaw;

    private boolean seekDirection = true;

    public MatchHeadingYaw(TurretYaw turretYaw){
        this.turretYaw = turretYaw;
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
            } else {
                this.turretYaw.setPositionSetpoint(targetYaw);
                return;
            }
        }
        this.turretYaw.setVelocitySetpoint(
            this.seekDirection ?
                Constants.Turret.TunedCoefficients.YawPID.kSeekAdjustment :
                -Constants.Turret.TunedCoefficients.YawPID.kSeekAdjustment
        );
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
