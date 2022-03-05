package frc.robot.subsystems.turret.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.turret.Turret;

public class ManualShoot extends CommandBase {
    private Turret turret;

    public ManualShoot(Turret turret){
        this.turret = turret;
    }

    @Override
    public void execute(){
        this.turret.flywheel.setFlywheelSpeed(0.8);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
