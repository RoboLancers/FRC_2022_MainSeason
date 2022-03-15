package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.turret.LaunchTrajectory;
import frc.robot.subsystems.turret.Turret;

public class UpperHubShoot extends CommandBase {
    private Turret turret;

    public UpperHubShoot(Turret turret){
        this.turret = turret;

        addRequirements(this.turret);
    }

    @Override
    public void execute(){
        LaunchTrajectory interpolatedTrajectory = this.turret.limelight.hasTarget() ?
            LaunchTrajectory.upperHubTrajectoryMap.interpolate(LaunchTrajectory.estimateDistance(this.turret.limelight.pitchOffset())) :
            LaunchTrajectory.upperHubTrajectoryMap.interpolate(Constants.Turret.Physics.kUpperHubRadius);

        SmartDashboard.putNumber("Distance XZ", LaunchTrajectory.estimateDistance(turret.limelight.pitchOffset()));

        SmartDashboard.putBoolean("Has Target", this.turret.limelight.hasTarget());
        SmartDashboard.putNumber("Interpolated Angle", interpolatedTrajectory.theta);
        SmartDashboard.putNumber("Interpolated Speed", interpolatedTrajectory.speed);

        // for tuning interpolation control points
        // double angle = SmartDashboard.getNumber("High Shot Angle", 0);
        // double speed = SmartDashboard.getNumber("High Shot Speed", 0);

        this.turret.pitch.setPosition(interpolatedTrajectory.theta);
        this.turret.flywheel.setVelocity(interpolatedTrajectory.speed);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
