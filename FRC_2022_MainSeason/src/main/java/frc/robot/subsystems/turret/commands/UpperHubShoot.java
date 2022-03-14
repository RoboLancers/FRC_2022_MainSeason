package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.LaunchTrajectory;
import frc.robot.subsystems.turret.Turret;

public class UpperHubShoot extends CommandBase {
    private Turret turret;

    public UpperHubShoot(Turret turret){
        this.turret = turret;
        this.turret.launchTrajectory = new LaunchTrajectory(
            SmartDashboard.getNumber("High Shot Angle", 0),
            SmartDashboard.getNumber("High Shot Speed", 0)
        );

        addRequirements(this.turret);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
