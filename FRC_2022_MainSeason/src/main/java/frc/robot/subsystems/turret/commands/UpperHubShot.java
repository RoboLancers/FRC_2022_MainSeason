package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.LaunchTrajectory;
import frc.robot.subsystems.turret.Turret;

public class UpperHubShot extends CommandBase {
    private Turret turret;

    private boolean tuning = false;

    public UpperHubShot(Turret turret){
        this.turret = turret;

        this.addRequirements(this.turret);
    }

    @Override
    public void execute(){
        double distance = this.turret.limelight.hasTarget() ? LaunchTrajectory.estimateDistance(this.turret.limelight.pitchOffset()) : 0;
        SmartDashboard.putNumber("Distance XZ", distance);

        if(this.tuning){
            double angle = SmartDashboard.getNumber("High Shot Angle", 0);
            double speed = SmartDashboard.getNumber("High Shot Speed", 0);
            this.turret.launchTrajectory = new LaunchTrajectory(angle, speed);
        } else {
            this.turret.launchTrajectory = LaunchTrajectory.upperHubTrajectoryMap.interpolate(distance);
            SmartDashboard.putNumber("Interpolated Angle", this.turret.launchTrajectory.theta);
            SmartDashboard.putNumber("Interpolated Speed", this.turret.launchTrajectory.speed);
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}