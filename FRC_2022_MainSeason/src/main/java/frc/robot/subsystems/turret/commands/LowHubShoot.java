package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;

public class LowHubShoot extends CommandBase {
    private Turret turret;

    public LowHubShoot(Turret turret){
        this.turret = turret;

        addRequirements(this.turret);
    }

    @Override
    public void execute(){
        double angle = 12; // SmartDashboard.getNumber("Low Shot Angle", 0);
        double speed = 1500; // SmartDashboard.getNumber("Low Shot Speed", 0);

        this.turret.pitch.setPosition(angle);
        this.turret.flywheel.setVelocity(speed);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
