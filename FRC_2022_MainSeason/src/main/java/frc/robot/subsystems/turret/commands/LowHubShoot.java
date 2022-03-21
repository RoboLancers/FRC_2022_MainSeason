package frc.robot.subsystems.turret.commands;

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
        this.turret.pitch.setPosition(12);
        this.turret.flywheel.setVelocity(1700);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}