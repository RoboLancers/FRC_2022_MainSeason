package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.LaunchTrajectory;
import frc.robot.subsystems.turret.Turret;

public class BasicShoot extends CommandBase {
    private Turret turret;

    public BasicShoot(Turret turret){
        this.turret = turret;
        this.turret.pitch.attemptingToZero = false;
    }

    @Override
    public void execute(){
        this.turret.launchTrajectory = new LaunchTrajectory(
            SmartDashboard.getNumber("Shoot Angle", 0.0),
            SmartDashboard.getNumber("Shoot Speed", 0.0)
        );
    }

    @Override
    public void end(boolean interrupted){
        this.turret.launchTrajectory = new LaunchTrajectory(0, 0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
