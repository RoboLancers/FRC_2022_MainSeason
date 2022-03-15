package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;

public class UpperHubShoot extends CommandBase {
    private Turret turret;

    public UpperHubShoot(Turret turret){
        this.turret = turret;

        addRequirements(this.turret);
    }

    @Override
    public void execute(){
        double angle = SmartDashboard.getNumber("High Shot Angle", 0);
        double speed = SmartDashboard.getNumber("High Shot Speed", 0);

        this.turret.pitch.setPosition(angle);
        this.turret.flywheel.setVelocity(speed);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
