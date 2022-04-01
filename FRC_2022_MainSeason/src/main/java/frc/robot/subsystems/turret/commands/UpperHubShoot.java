package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.LaunchTrajectory;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.LaunchTrajectory.InterpolationTable;
import frc.robot.util.XboxController;

public class UpperHubShoot extends CommandBase {
    private Turret turret;


    public UpperHubShoot(Turret turret){
        this.turret = turret;

        addRequirements(this.turret);
    }

    @Override
    public void execute(){
        SmartDashboard.putBoolean("Has Target", this.turret.limelight.hasTarget());
        SmartDashboard.putNumber("Distance XZ", LaunchTrajectory.estimateDistance(turret.limelight.pitchOffset()));

        if(SmartDashboard.getBoolean("Use Interpolation", true)){
            InterpolationTable upperHubTrajectoryMap = LaunchTrajectory.getUpperHubTrajectoryMap();

            LaunchTrajectory interpolatedTrajectory = upperHubTrajectoryMap.interpolate(
                this.turret.limelight.hasTarget() ? LaunchTrajectory.estimateDistance(this.turret.limelight.pitchOffset()) : 0
            );

            SmartDashboard.putNumber("Interpolated Angle", interpolatedTrajectory.theta);
            SmartDashboard.putNumber("Interpolated Speed", interpolatedTrajectory.speed);

            this.turret.pitch.setPosition(interpolatedTrajectory.theta);
            this.turret.flywheel.setVelocity(interpolatedTrajectory.speed);
        } else {
            double angle = SmartDashboard.getNumber("High Shot Angle", 0);
            double speed = SmartDashboard.getNumber("High Shot Speed", 0);

            this.turret.pitch.setPosition(angle);
            this.turret.flywheel.setVelocity(speed);
        }
    }

    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
