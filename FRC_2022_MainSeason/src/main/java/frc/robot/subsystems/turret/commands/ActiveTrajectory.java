package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.turret.Trajectory;
import frc.robot.subsystems.turret.Turret;

public class ActiveTrajectory extends CommandBase {
    private Turret turret;

    private static double minDeltaYaw = 1.00;

    // Recorded yaw on last execution 
    private double lastYaw;

    public ActiveTrajectory(Turret turret){
        this.turret = turret;

        this.lastYaw = this.turret.getYaw();
    }

    @Override
    public void execute(){
        if(
            Math.abs(this.turret.getYaw() - this.lastYaw) > minDeltaYaw
        ){
            this.lastYaw = this.turret.getYaw();
            if(this.turret.limelight.hasTarget()){
                this.turret.trajectory = Trajectory.usingAlphaImpact(
                    Constants.Turret.FieldInfo.g,
                    Trajectory.estimateDistance(
                        Constants.Turret.FieldInfo.turretShotDeltaY,
                        this.turret.limelight.yawOffset(),
                        this.turret.limelight.pitchOffset()
                    ),
                    Constants.Turret.FieldInfo.turretShotDeltaY,
                    Constants.Turret.FieldInfo.alpha
                );
            } else {
                this.turret.trajectory = new Trajectory(0, 0, 0);
            }
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
