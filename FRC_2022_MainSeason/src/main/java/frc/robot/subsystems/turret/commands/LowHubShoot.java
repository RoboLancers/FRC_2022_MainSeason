package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.LaunchTrajectory;
import frc.robot.subsystems.turret.Turret;

public class LowHubShoot extends CommandBase {
    private Turret turret;

    public LowHubShoot(Turret turret){
        this.turret = turret;
        this.turret.launchTrajectory = new LaunchTrajectory(
            SmartDashboard.getNumber("Low Shot Angle", 0),
            SmartDashboard.getNumber("Low Shot Speed", 0)
        );

        addRequirements(this.turret);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
